#![no_std]
#[cfg(feature = "blocking")]
use embedded_hal as hal;

#[cfg(feature = "async")]
use embedded_hal_async as hal;

use hal::spi::Operation;

mod register;
use register::Register;

const READ_MASK: u8 = 0x80;
const ID_ICM42688: u8 = 0x47;
const FIFO_CONFIG: u8 = 0x07;

#[allow(dead_code)]
const CLIP_LIMIT: f32 = 15.5 * 9.80665f32;
#[allow(dead_code)]
const TEMP_SENSITIVITY: f32 = 1.0 / 2.07;

pub struct ICM42688<SPI> {
    spi: SPI,
}

#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct FIFOData {
    pub header: u8,
    pub accel: [i16; 3],
    pub gyro: [i16; 3],
    pub temperature: i8,
    pub timestamp: u16,
}

impl FIFOData {
    pub fn default() -> Self {
        Self {
            header: 0,
            accel: [0; 3],
            gyro: [0; 3],
            temperature: 0,
            timestamp: 0,
        }
    }

    pub fn check_header(&self) -> bool {
        if (self.header & 0xFC) != 0x68 {
            return false;
        }
        return true;
    }
}

const SAMPLE_SIZE: usize = core::mem::size_of::<FIFOData>();
const GRAVITY_MSS: f32 = 9.80665;
const SCALE_RANGE_16BIT: f32 = 32768.0;
const ACCEL_SCALE_16G: f32 = GRAVITY_MSS / (SCALE_RANGE_16BIT / 16.0);
const GYRO_SCALE_2000DPS: f32 =
    (3.141592653589793238462643383279502884 / 180.0) / (SCALE_RANGE_16BIT / 2000.0);

pub fn apply_scales(acc: &mut nalgebra::Vector3<f32>, gyr: &mut nalgebra::Vector3<f32>) {
    *acc *= ACCEL_SCALE_16G;
    *gyr *= GYRO_SCALE_2000DPS;
}

#[cfg(feature = "async")]
mod asynch {
    use super::*;
    impl<SPI: hal::spi::SpiDevice> ICM42688<SPI> {
        async fn read_register(&mut self, reg: Register) -> Result<u8, Error<SPI::Error>> {
            let mut buf = [0; 1];
            self.spi
                .transaction(&mut [
                    Operation::Write(&[reg.addr() | READ_MASK]),
                    Operation::Read(&mut buf),
                ])
                .await
                .map_err(Error::Spi)?;
            Ok(buf[0])
        }

        async fn read_registers(
            &mut self,
            reg: Register,
            buf: &mut [u8],
        ) -> Result<(), Error<SPI::Error>> {
            self.spi
                .transaction(&mut [
                    Operation::Write(&[reg.addr() | READ_MASK]),
                    Operation::Read(buf),
                ])
                .await
                .map_err(Error::Spi)?;

            Ok(())
        }

        pub async fn get_sample_num(&mut self) -> Result<u16, Error<SPI::Error>> {
            let mut buf = [0u8; 2];
            self.read_registers(Register::FifoCounth, &mut buf).await?;
            Ok(u16::from_le_bytes(buf))
        }

        pub async fn get_n_samples(
            &mut self,
            data: &mut [FIFOData],
        ) -> Result<(), Error<SPI::Error>> {
            let bytes = unsafe {
                core::slice::from_raw_parts_mut(
                    core::mem::transmute::<*const FIFOData, *mut u8>(data.as_ptr()),
                    SAMPLE_SIZE * data.len(),
                )
            };
            self.read_registers(Register::FifoData, bytes).await?;
            Ok(())
        }

        async fn read_register_bank(
            &mut self,
            bank: u8,
            reg: Register,
        ) -> Result<u8, Error<SPI::Error>> {
            self.write_register(Register::BankSel, bank).await?;
            let res = self.read_register(reg).await?;
            self.write_register(Register::BankSel, 0).await?;
            Ok(res)
        }

        async fn write_register_bank(
            &mut self,
            bank: u8,
            reg: Register,
            val: u8,
        ) -> Result<(), Error<SPI::Error>> {
            self.write_register(Register::BankSel, bank).await?;
            self.write_register(reg, val).await?;
            self.write_register(Register::BankSel, 0).await?;
            Ok(())
        }

        async fn write_register(
            &mut self,
            reg: Register,
            val: u8,
        ) -> Result<(), Error<SPI::Error>> {
            let buf = [reg.addr(), val];
            self.spi.write(&buf).await.map_err(Error::Spi)?;
            Ok(())
        }

        pub async fn check_id(&mut self) -> Result<bool, Error<SPI::Error>> {
            self.read_register(Register::WhoAmI).await.map(|v| match v {
                ID_ICM42688 => true,
                _ => false,
            })
        }

        pub async fn new(spi: SPI) -> Result<Self, Error<SPI::Error>> {
            let mut dev = Self { spi };

            let ans = dev.check_id().await?;

            if ans {
                return Ok(dev);
            }

            Err(Error::IdNotMatch)
        }

        pub async fn fifo_reset(&mut self) -> Result<(), Error<SPI::Error>> {
            self.write_register(Register::FifoConfig, 0x80).await?;
            self.write_register(Register::FifoConfig1, FIFO_CONFIG)
                .await?;
            self.write_register(Register::IntfConfig0, 0xC0).await?;
            self.write_register(Register::SignalPathReset, 2).await?;
            Ok(())
        }

        pub async fn set_filter_and_scaling<D: hal::delay::DelayNs>(
            &mut self,
            mut delayer: D,
        ) -> Result<(), Error<SPI::Error>> {
            let odr_config = 0x06; //1KHz sampling
            let aaf_delt = 6u8;
            let accel_aaf_delt = 5u8;
            let aaf_deltsqr = 36u16;
            let accel_aaf_deltsqr = 25u16;
            let aaf_bitshift = 10u8;
            let accel_aaf_bitshift = 10u8;

            self.write_register(Register::PwrMgmt0, 0x0F).await?;

            delayer.delay_us(300).await;

            self.write_register(Register::GyroConfig0, odr_config)
                .await?;
            self.write_register(Register::AccelConfig0, odr_config)
                .await?;
            let aaf_enable = self
                .read_register_bank(1, Register::GyroConfigStatic2)
                .await?;
            self.write_register_bank(1, Register::GyroConfigStatic2, aaf_enable & !0x03)
                .await?;
            self.write_register_bank(1, Register::GyroConfigStatic3, aaf_delt)
                .await?;
            self.write_register_bank(1, Register::GyroConfigStatic4, (aaf_deltsqr & 0xFF) as u8)
                .await?;
            self.write_register_bank(1, Register::GyroConfigStatic5, (aaf_bitshift << 4) & 0xF0)
                .await?;

            self.write_register_bank(2, Register::AccelConfigStatic2, accel_aaf_delt << 1)
                .await?;
            self.write_register_bank(
                2,
                Register::AccelConfigStatic3,
                (accel_aaf_deltsqr & 0xFF) as u8,
            )
            .await?;
            self.write_register_bank(
                2,
                Register::AccelConfigStatic4,
                ((accel_aaf_bitshift << 4) & 0xF0) | ((accel_aaf_deltsqr >> 8) & 0xF0) as u8,
            )
            .await?;

            Ok(())
        }
    }
}

#[allow(unused_imports)]
#[cfg(feature = "async")]
use asynch::*;

#[derive(Copy, Clone, Debug)]
pub enum Error<SPI> {
    Spi(SPI),
    IdNotMatch,
    Debug(u8),
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {}
}
