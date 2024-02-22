use super::fmt;
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::{bind_interrupts, interrupt, peripherals, sdmmc, time, usart, usb_otg};
use serial_manager::{self};

bind_interrupts!(struct Irqs {
    UART7 => usart::InterruptHandler<peripherals::UART7>;
    OTG_FS => usb_otg::InterruptHandler<peripherals::USB_OTG_FS>;
    SDMMC1 => sdmmc::InterruptHandler<peripherals::SDMMC1>;
});

fn config() -> embassy_stm32::Config {
    use embassy_stm32::rcc::*;
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(embassy_stm32::rcc::Hse {
        mode: embassy_stm32::rcc::HseMode::Oscillator,
        freq: time::mhz(8),
    });
    config.rcc.hsi48 = Some(embassy_stm32::rcc::Hsi48Config {
        sync_from_usb: true,
    }); // needed for USB
    config.rcc.hsi = None;
    config.rcc.ls = LsConfig {
        rtc: RtcClockSource::DISABLE,
        lsi: false,
        lse: None,
    };
    config.rcc.csi = true;
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL100,
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV8),
        divr: None,
    });
    config.rcc.pll2 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL45,
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV5),
        divr: Some(PllDiv::DIV1),
    });
    config.rcc.pll3 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV2,
        mul: PllMul::MUL72,
        divp: Some(PllDiv::DIV3),
        divq: Some(PllDiv::DIV6),
        divr: Some(PllDiv::DIV9),
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.d1c_pre = AHBPrescaler::DIV1;
    config.rcc.ahb_pre = AHBPrescaler::DIV2;
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config.rcc.apb3_pre = APBPrescaler::DIV2;
    config.rcc.apb4_pre = APBPrescaler::DIV2;
    config.rcc.voltage_scale = VoltageScale::Scale1;

    #[cfg(feature = "defmt")]
    {
        config.enable_debug_during_sleep = true;
    }
    #[cfg(not(feature = "defmt"))]
    {
        config.enable_debug_during_sleep = false;
    }
    config
}

static EXECUTOR_HIGH: embassy_executor::InterruptExecutor =
    embassy_executor::InterruptExecutor::new();

static EXECUTOR_MED: embassy_executor::InterruptExecutor =
    embassy_executor::InterruptExecutor::new();

impl super::GenericBoard for super::Board {
    fn init() -> (embassy_executor::SendSpawner, embassy_executor::SendSpawner) {
        let _ = embassy_stm32::init(config());
        interrupt::UART4.set_priority(interrupt::Priority::P2);

        serial0_bind();
        serial1_bind();

        let executor_high = EXECUTOR_HIGH.start(interrupt::UART4);
        let executor_med = EXECUTOR_MED.start(interrupt::UART5);

        unsafe { cortex_m::interrupt::enable() };
        (executor_high, executor_med)
    }
}

fn serial1_bind() {
    let sc1: &'static mut serial_manager::SerialRingBuf =
        static_cell::make_static!(ringbuf::StaticRb::default());

    let sc2: &'static mut serial_manager::SerialRingBuf =
        static_cell::make_static!(ringbuf::StaticRb::default());
    let (p1, c1) = sc1.split_ref();
    let (p2, c2) = sc2.split_ref();
    serial_manager::bind_port(serial_manager::Protocol::MavlinkV2, 1, c1, p1, c2, p2);
}

fn serial0_bind() {
    let sc1: &'static mut serial_manager::SerialRingBuf =
        static_cell::make_static!(ringbuf::StaticRb::default());

    let sc2: &'static mut serial_manager::SerialRingBuf =
        static_cell::make_static!(ringbuf::StaticRb::default());
    let (p1, c1) = sc1.split_ref();
    let (p2, c2) = sc2.split_ref();
    serial_manager::bind_port(serial_manager::Protocol::MavlinkV2, 0, c1, p1, c2, p2);
}

pub mod hw_tasks {
    use embassy_usb::driver::EndpointError;
    use embedded_io_async::Write;

    use super::*;

    #[allow(unused)]
    async fn port_read<T: usart::BasicInstance, R: usart::RxDma<T>>(
        mut port: usart::UartRx<'_, T, R>,
        mut rb_producer: serial_manager::RingBufWriteRef,
        name: &str,
    ) {
        let mut buf = [0u8; 100];
        loop {
            let res = port
                .read_until_idle(&mut buf)
                .await
                .map_err(|e| fmt::debug!("{}, read err {}", name, e));
            if let Ok(v) = res {
                let amt = core::cmp::min(rb_producer.free_len(), v);
                rb_producer.push_slice(&buf[..amt]);
                if amt < v {
                    fmt::debug!("{}, RBfull. written {}, requested {}", name, amt, v)
                }
            }
        }
    }

    #[allow(unused)]
    async fn port_write<T: usart::BasicInstance, R: usart::TxDma<T>>(
        mut port: usart::UartTx<'_, T, R>,
        mut consumer: serial_manager::RingBufReadRef,
        name: &str,
    ) {
        let mut buf = [0u8; 100];
        loop {
            if consumer.len() == 0 {
                embassy_time::Timer::after_micros(5).await;
                continue;
            }

            let amt = consumer.pop_slice(&mut buf);
            fmt::debug!("{} writing len bytes {}", name, amt);
            let _ = port
                .write_all(&buf[..amt])
                .await
                .map_err(|e| fmt::debug!("{}, write error {}", name, e));
        }
    }

    #[embassy_executor::task]
    pub async fn serial0_runner(_cfg: serial_manager::Config) {
        let p = unsafe { embassy_stm32::Peripherals::steal() };
        let (mut consumer, mut producer) = fmt::unwrap!(serial_manager::find_port_rb_ref(0));

        let name = "Serial0";

        // Create the driver, from the HAL.
        let mut ep_out_buffer = [0u8; 2048];
        let mut config = usb_otg::Config::default();
        config.vbus_detection = false;
        let driver = usb_otg::Driver::new_fs(
            p.USB_OTG_FS,
            Irqs,
            p.PA12,
            p.PA11,
            &mut ep_out_buffer,
            config,
        );
        // Create embassy-usb Config
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("41mo");
        config.product = Some("USB-serial");
        config.serial_number = Some("12345678");
        // Required for windows compatibility.
        // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
        config.device_class = 0xEF;
        config.device_sub_class = 0x02;
        config.device_protocol = 0x01;
        config.composite_with_iads = true;
        // Create embassy-usb DeviceBuilder using the driver and config.
        // It needs some buffers for building the descriptors.
        let mut device_descriptor = [0; 256];
        let mut config_descriptor = [0; 256];
        let mut bos_descriptor = [0; 256];
        let mut control_buf = [0; 64];
        let mut state = embassy_usb::class::cdc_acm::State::new();

        let mut builder = embassy_usb::Builder::new(
            driver,
            config,
            &mut device_descriptor,
            &mut config_descriptor,
            &mut bos_descriptor,
            &mut [], // no msos descriptors
            &mut control_buf,
        );

        // Create classes on the builder.
        let class = embassy_usb::class::cdc_acm::CdcAcmClass::new(&mut builder, &mut state, 64);

        // Build the builder.
        let mut usb = builder.build();

        let usb_runner = usb.run();

        let (mut tx, mut rx) = class.split();

        let read = async move {
            let mut buf = [0u8; 128];
            loop {
                rx.wait_connection().await;
                loop {
                    let l = match rx.read_packet(&mut buf).await {
                        Ok(l) => l,
                        Err(e) => match e {
                            EndpointError::BufferOverflow => panic!("{} buffer overflow", name),
                            EndpointError::Disabled => break,
                        },
                    };
                    let amt = core::cmp::min(producer.free_len(), l);
                    producer.push_slice(&buf[..amt]);
                    if amt < l {
                        fmt::debug!("{}, RBfull. written {}, requested {}", name, amt, l)
                    }
                }
            }
        };

        let write = async move {
            let mut buf = [0u8; 64];
            loop {
                tx.wait_connection().await;
                loop {
                    if consumer.is_full() {
                        consumer.clear();
                    }
                    if consumer.len() == 0 {
                        embassy_time::Timer::after_micros(5).await;
                        continue;
                    }
                    let ps = tx.max_packet_size();
                    let amt = consumer.pop_slice(&mut buf[..ps as usize]);

                    let r = tx.write_packet(&buf[..amt]).await;
                    if let Err(e) = r {
                        match e {
                            EndpointError::BufferOverflow => panic!("{} buffer overflow", name),
                            EndpointError::Disabled => break,
                        }
                    }
                }
            }
        };

        embassy_futures::join::join3(usb_runner, read, write).await;
    }

    #[embassy_executor::task]
    pub async fn serial1_runner(cfg: serial_manager::Config) {
        let mut uc = usart::Config::default();
        uc.baudrate = cfg.baud;

        let p = unsafe { embassy_stm32::Peripherals::steal() };
        let port = fmt::unwrap!(usart::Uart::new(
            p.UART7, p.PE7, p.PE8, Irqs, p.DMA1_CH0, p.DMA1_CH1, uc
        ));
        let (consumer, producer) = fmt::unwrap!(serial_manager::find_port_rb_ref(1));

        let (tx, rx) = port.split();
        let name = "Serial1";

        let reader = port_read(rx, producer, name);
        let writer = port_write(tx, consumer, name);

        embassy_futures::join::join(reader, writer).await;
    }

    #[interrupt]
    unsafe fn UART4() {
        EXECUTOR_HIGH.on_interrupt()
    }

    #[interrupt]
    unsafe fn UART5() {
        EXECUTOR_MED.on_interrupt()
    }

    #[cfg(feature = "imu")]
    #[embassy_executor::task]
    pub async fn imu_task() {
        use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
        use embassy_stm32::spi;
        use embassy_sync::blocking_mutex::raw::NoopRawMutex;
        use embassy_sync::mutex::Mutex;

        let mut rb = imu::get_imu_producer(imu::ImuInstance::IMU1).unwrap();

        let mut spi_config = spi::Config::default();
        spi_config.mode = spi::MODE_3;
        spi_config.bit_order = spi::BitOrder::MsbFirst;
        spi_config.frequency = time::mhz(8);
        let p = unsafe { embassy_stm32::Peripherals::steal() };

        let cs_pin = embassy_stm32::gpio::Output::new(
            p.PC15,
            embassy_stm32::gpio::Level::High,
            embassy_stm32::gpio::Speed::Low,
        );

        let spi = spi::Spi::new(
            p.SPI1, p.PA5, p.PD7, p.PA6, p.DMA1_CH5, p.DMA1_CH4, spi_config,
        );
        static SPI_BUS: static_cell::StaticCell<
            Mutex<
                NoopRawMutex,
                spi::Spi<peripherals::SPI1, peripherals::DMA1_CH5, peripherals::DMA1_CH4>,
            >,
        > = static_cell::StaticCell::new();

        let bus = SPI_BUS.init(Mutex::new(spi));
        let dev = SpiDevice::new(bus, cs_pin);
        let mut dev = icm42688::ICM42688::new(dev).await.unwrap();
        dev.fifo_reset().await.unwrap();
        dev.set_filter_and_scaling(embassy_time::Delay)
            .await
            .unwrap();
        const FIFO_BUFF_LEN: usize = 8;
        let mut buff = [icm42688::FIFOData::default(); FIFO_BUFF_LEN];

        let mut accumulated_accel = nalgebra::Vector3::<f32>::default();
        let mut accumulated_gyro = nalgebra::Vector3::<f32>::default();
        let mut accumulated_dt_100us: u16 = 0;
        let mut accumulator: usize = 0;
        let mut last_run = embassy_time::Instant::now();

        let mut adjust: i32 = 0;
        let loop_dt: u64 = 1_000_000 / 2000;
        loop {
            let now = embassy_time::Instant::now();
            let dt = (now - last_run).as_micros();
            let mut n_samples = dev.get_sample_num().await.unwrap();

            // adjust timer to always get 1 sample
            if n_samples == 1 {
                ()
            } else if n_samples == 0 {
                adjust += 1;
            } else {
                adjust -= 1
            }

            'outer: while n_samples > 0 {
                let n = core::cmp::min(n_samples as usize, FIFO_BUFF_LEN);
                dev.get_n_samples(&mut buff[..n]).await.unwrap();

                for i in buff[..n].iter() {
                    if !i.check_header() {
                        fmt::debug!("header err");
                        break 'outer;
                    }

                    let mut acc = nalgebra::Vector3::<f32>::new(
                        -1.0 * i.accel[1] as f32,
                        -1.0 * i.accel[0] as f32,
                        -1.0 * (i.accel[2] as f32),
                    );
                    let mut gyr = nalgebra::Vector3::<f32>::new(
                        -1.0 * i.gyro[1] as f32,
                        -1.0 * i.gyro[0] as f32,
                        -1.0 * (i.gyro[2] as f32),
                    );
                    icm42688::apply_scales(&mut acc, &mut gyr);
                    accumulated_accel += acc;
                    accumulated_gyro += gyr;
                    accumulated_dt_100us += (dt / 100) as u16;
                    accumulator += 1;

                    // let mut lacc = [0.0; 3];
                    // let mut lgyr = [0.0; 3];
                    // lacc.copy_from_slice(acc.as_slice());
                    // lgyr.copy_from_slice(gyr.as_slice());
                    // logger::log_msg(logger::LogMessages::LogImuRaw(logger::ImuRaw {
                    //     sample_time: embassy_time::Instant::now().as_micros(),
                    //     acc: lacc,
                    //     gyr: lgyr,
                    // }));
                }
                n_samples -= n as u16;
            }

            if accumulator >= 10 {
                accumulated_accel /= accumulator as f32;
                accumulated_gyro /= accumulator as f32;
                accumulated_dt_100us /= accumulator as u16;

                let _ = rb.push((
                    accumulated_accel,
                    accumulated_gyro,
                    accumulated_dt_100us / 10, // convert to micros
                ));

                accumulator = 0;
                accumulated_accel *= 0.0;
                accumulated_gyro *= 0.0;
            }

            let next_run =
                now + embassy_time::Duration::from_micros((loop_dt as i32 + adjust) as u64);

            last_run = now;

            // crate::fmt::debug!(
            //     "hz: {}, dt {}, adjust {}",
            //     1_000_000 / core::cmp::max(dt, 1),
            //     dt,
            //     adjust
            // );

            embassy_time::Timer::at(next_run).await;
        }
    }

    #[embassy_executor::task]
    pub async fn pwm_task() {
        use embassy_stm32::time::hz;
        use embassy_stm32::timer::complementary_pwm;
        use embassy_stm32::timer::simple_pwm;
        use embassy_stm32::timer::Channel;
        use servo_control::PwmDriverMsg;

        let p = unsafe { embassy_stm32::Peripherals::steal() };
        /*
         * servo duty cycle
         * for med we need 1.5 ms so for 20ms is 7.5% duty cycle
         */

        /*
         *
         * PB0  TIM8_CH2N TIM8  PWM(1)  GPIO(50)
         * PB1  TIM8_CH3N TIM8  PWM(2)  GPIO(51)
         * PA0  TIM5_CH1  TIM5  PWM(3)  GPIO(52)
         * PA1  TIM5_CH2  TIM5  PWM(4)  GPIO(53)
         * PA2  TIM5_CH3  TIM5  PWM(5)  GPIO(54)
         * PA3  TIM5_CH4  TIM5  PWM(6)  GPIO(55)
         * PD12 TIM4_CH1 TIM4 PWM(7) GPIO(56)
         * PD13 TIM4_CH2 TIM4 PWM(8) GPIO(57)
         * PD14 TIM4_CH3 TIM4 PWM(9) GPIO(58)
         * PD15 TIM4_CH4 TIM4 PWM(10) GPIO(59)
         * PE5  TIM15_CH1 TIM15 PWM(11) GPIO(60)
         * PE6  TIM15_CH2 TIM15 PWM(12) GPIO(61)
         * PA8 TIM1_CH1 TIM1 PWM(13) GPIO(62) # for WS2812 LED
         */

        // const max_servo_period_us:f32 = 2000.0;
        // const med_servo_period_us:f32 = 1500.0;
        // const min_servo_period_us:f32 = 1000.0;
        const DUTY_CYCLE_US: f32 = 20000.0; // this is
        let servo_reciever = servo_control::get_receiver();
        let mut pwmdriver1 = complementary_pwm::ComplementaryPwm::new(
            p.TIM8,
            None,
            None,
            None,
            Some(complementary_pwm::ComplementaryPwmPin::new_ch2(
                p.PB0,
                embassy_stm32::gpio::OutputType::PushPull,
            )), // PWM1
            None,
            Some(complementary_pwm::ComplementaryPwmPin::new_ch3(
                p.PB1,
                embassy_stm32::gpio::OutputType::PushPull,
            )), // PWM2
            None,
            None,
            hz(50),
            Default::default(),
        );
        let mut pwmdriver2 = simple_pwm::SimplePwm::new(
            p.TIM5,
            Some(simple_pwm::PwmPin::new_ch1(
                p.PA0,
                embassy_stm32::gpio::OutputType::PushPull,
            )), // PWM3
            Some(simple_pwm::PwmPin::new_ch2(
                p.PA1,
                embassy_stm32::gpio::OutputType::PushPull,
            )), // PWM4
            Some(simple_pwm::PwmPin::new_ch3(
                p.PA2,
                embassy_stm32::gpio::OutputType::PushPull,
            )), // PWM5
            Some(simple_pwm::PwmPin::new_ch4(
                p.PA3,
                embassy_stm32::gpio::OutputType::PushPull,
            )), // PWM6
            hz(50),
            Default::default(),
        );
        let drv1_max_duty = pwmdriver1.get_max_duty() as f32;
        let drv2_max_duty = pwmdriver2.get_max_duty() as f32;
        let pwmdriver1 = &mut pwmdriver1;
        let pwmdriver2 = &mut pwmdriver2;

        let srv_to_ch_map = [
            (Channel::Ch2),
            (Channel::Ch3),
            (Channel::Ch1),
            (Channel::Ch2),
            (Channel::Ch3),
            (Channel::Ch4),
        ];

        loop {
            let v = servo_reciever.receive().await;
            match v {
                PwmDriverMsg::SetPeriod(r) => {
                    let idx = r.0 as usize - 1;
                    let pwm_ch = srv_to_ch_map[idx];
                    let c = r.1 as f32 / DUTY_CYCLE_US;
                    match idx {
                        0..=1 => {
                            pwmdriver1.set_duty(pwm_ch, (drv1_max_duty * c) as u16);
                        }
                        2..=5 => {
                            fmt::debug!("setting {} to {}", idx, r.1);
                            pwmdriver2.set_duty(pwm_ch, (drv2_max_duty * c) as u16);
                        }
                        _ => fmt::debug!("wrong channel"),
                    }
                }
                PwmDriverMsg::OnOff(r) => {
                    let idx = r.0 as usize - 1;
                    let pwm_ch = srv_to_ch_map[idx];
                    match idx {
                        0..=1 => {
                            if r.1 {
                                pwmdriver1.enable(pwm_ch)
                            } else {
                                pwmdriver1.disable(pwm_ch)
                            }
                        }
                        2..=5 => {
                            if r.1 {
                                fmt::debug!("enabling channel {}", idx);
                                pwmdriver2.enable(pwm_ch)
                            } else {
                                pwmdriver2.disable(pwm_ch)
                            }
                        }
                        _ => fmt::debug!("wrong channel"),
                    }
                }
            }
        }
    }
    #[cfg(feature="sdcard")]
    mod sdcard {
        use super::*;
        use embedded_fatfs::{FileSystem, FsOptions};
        use embedded_io_async::{ErrorType, Read, Seek, SeekFrom};

        struct SdmmcInmemStorage<'a, T: sdmmc::Instance> {
            block: sdmmc::DataBlock,
            drv: sdmmc::Sdmmc<'a, T>,
            block_idx: u32,
            offset_in_block: u16,
            max_blocks: u32,
            should_read: bool,
        }

        impl<'a, T: sdmmc::Instance> SdmmcInmemStorage<'a, T> {
            fn new(driver: sdmmc::Sdmmc<'a, T>) -> Self {
                Self {
                    block: sdmmc::DataBlock([0u8; 512]),
                    block_idx: 0,
                    offset_in_block: 0,
                    max_blocks: driver.card().unwrap().csd.block_count(),
                    drv: driver,
                    should_read: true,
                }
            }

            async fn retry_write(&mut self) -> Result<(), Error> {
                let mut retry = false;
                loop {
                    if let Err(e) = self.drv.write_block(self.block_idx, &self.block).await {
                        match e {
                            sdmmc::Error::Timeout => retry = true,
                            _ => Err(e)?,
                        }
                    } else {
                        retry = false;
                    }
                    if !retry {
                        break;
                    }
                }
                Ok(())
            }
        }

        #[derive(Debug, defmt::Format)]
        enum Error {
            SdmmcErr(sdmmc::Error),
        }

        impl From<sdmmc::Error> for Error {
            fn from(value: sdmmc::Error) -> Self {
                Error::SdmmcErr(value)
            }
        }

        const BLOCK_SIZE: u16 = 512;

        impl embedded_io_async::Error for Error {
            fn kind(&self) -> embedded_io_async::ErrorKind {
                todo!()
            }
        }

        impl<'a, T: sdmmc::Instance> embedded_io_async::ErrorType for SdmmcInmemStorage<'a, T> {
            type Error = Error;
        }

        impl<'a, T: sdmmc::Instance> embedded_io_async::Read for SdmmcInmemStorage<'a, T> {
            async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
                if self.should_read {
                    self.drv
                        .read_block(self.block_idx, &mut self.block)
                        .await
                        .unwrap();
                    self.should_read = false;
                }

                let bytes_copied;
                let new_offset: usize =
                    <u16 as Into<usize>>::into(self.offset_in_block) + buf.len();

                let bs_usize: usize = BLOCK_SIZE.into();
                if (new_offset / bs_usize) > 0 {
                    // we are crossing bound of block
                    bytes_copied = (BLOCK_SIZE - self.offset_in_block).into();
                    buf[..bytes_copied]
                        .copy_from_slice(&self.block.0[self.offset_in_block.into()..]);
                    self.block_idx += 1;
                    self.offset_in_block = 0;
                    self.should_read = true;
                    assert!(bytes_copied == buf.len())
                } else {
                    // we are inside of block
                    buf.copy_from_slice(
                        &self.block.0[self.offset_in_block.into()..new_offset.into()],
                    );
                    self.offset_in_block = new_offset.try_into().unwrap();
                    bytes_copied = buf.len();
                }

                Ok(bytes_copied)
            }
        }

        impl<'a, T: sdmmc::Instance> embedded_io_async::Write for SdmmcInmemStorage<'a, T> {
            async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                let new_offset = <u16 as Into<usize>>::into(self.offset_in_block) + buf.len();
                #[cfg(sddebug)]
                fmt::debug!("write  bytes: {}", buf.len());

                // let bytes_offset:u64 = <u32 as Into<u64>>::into(self.block_idx)*512 + <u16 as Into<u64>>::into(self.offset_in_block.into());
                // fmt::debug!(":#x?", bytes_offset);

                if self.offset_in_block == 0
                    && new_offset / <u16 as Into<usize>>::into(BLOCK_SIZE) > 0
                {
                    let block = sdmmc::DataBlock(unsafe { *(buf.as_ptr() as *const [u8; 512]) });
                    self.drv.write_block(self.block_idx, &block).await.unwrap();
                    self.block_idx += 1;
                    self.should_read = true;
                    return Ok(BLOCK_SIZE.into());
                }

                if self.should_read {
                    self.drv
                        .read_block(self.block_idx, &mut self.block)
                        .await
                        .unwrap();
                    self.should_read = false;
                }

                let updated_bytes_size;
                let bs_usize: usize = BLOCK_SIZE.into();
                if (new_offset / bs_usize) > 0 {
                    updated_bytes_size = (BLOCK_SIZE - self.offset_in_block).into();
                    self.block.0[self.offset_in_block.into()..]
                        .copy_from_slice(&buf[..updated_bytes_size]);
                    self.retry_write().await?;
                    self.block_idx += 1;
                    self.offset_in_block = 0;
                    self.should_read = true;
                } else {
                    self.block.0[self.offset_in_block.into()..new_offset.into()]
                        .copy_from_slice(buf);
                    self.offset_in_block = new_offset.try_into().unwrap();
                    updated_bytes_size = buf.len();
                    self.retry_write().await?;
                }
                assert!(self.block_idx < self.max_blocks);
                Ok(updated_bytes_size)
            }
        }

        impl<'a, T: sdmmc::Instance> embedded_io_async::Seek for SdmmcInmemStorage<'a, T> {
            async fn seek(&mut self, pos: embedded_io_async::SeekFrom) -> Result<u64, Self::Error> {
                match pos {
                    embedded_io_async::SeekFrom::Start(v) => {
                        let mb: u64 = self.max_blocks.into();
                        assert!(v < mb * 512);
                        let bs: u64 = BLOCK_SIZE.into();
                        self.block_idx = (v / bs).try_into().unwrap();
                        self.offset_in_block = (v % bs).try_into().unwrap();
                        let r = <u32 as Into<u64>>::into(self.block_idx) * 512
                            + <u16 as Into<u64>>::into(self.offset_in_block);
                        assert!(r == v);
                        #[cfg(sddebug)]
                        fmt::trace!(
                            "seeking to addr {:x} from start , new block idx {}, new offset {}",
                            v,
                            self.block_idx,
                            self.offset_in_block
                        );
                    }
                    embedded_io_async::SeekFrom::End(v) => {
                        let bs: i64 = BLOCK_SIZE.into();
                        self.block_idx = self
                            .max_blocks
                            .checked_add_signed((v / bs).try_into().unwrap())
                            .unwrap();
                        self.offset_in_block = 0;
                        let neg_offs: u16 = (v % bs).unsigned_abs().try_into().unwrap();

                        if neg_offs > 0 {
                            self.block_idx -= 1;
                            self.offset_in_block = 512 - neg_offs;
                        }
                    }
                    embedded_io_async::SeekFrom::Current(v) => {
                        let c: i64 = <u32 as Into<i64>>::into(self.block_idx * 512)
                            + <u16 as Into<i64>>::into(self.offset_in_block);
                        #[cfg(sddebug)]
                        fmt::trace!("seeking from cur with {}B; current {}B", v, c);
                        assert!(c + v >= 0);
                        let bs: i64 = BLOCK_SIZE.into();
                        let upd = self
                            .block_idx
                            .checked_add_signed((v / bs).try_into().unwrap())
                            .unwrap();
                        self.block_idx = upd;
                        let upd = (v % bs).unsigned_abs().try_into().unwrap();
                        self.offset_in_block = upd;
                    }
                }
                self.should_read = true;
                let r = <u32 as Into<u64>>::into(self.block_idx) * 512
                    + <u16 as Into<u64>>::into(self.offset_in_block);
                Ok(r)
            }
        }

        const LOG_NAME_MAX_BYTES: usize = 9;
        fn get_logname(buff: &[u8]) -> heapless::String<LOG_NAME_MAX_BYTES> {
            use core::str::FromStr;
            if buff.len() < LOG_NAME_MAX_BYTES {
                return heapless::String::from_str("00001.LOG").unwrap();
            }

            let name = core::str::from_utf8(&buff);
            let name = match name {
                Ok(v) => v,
                Err(_) => return heapless::String::from_str("00001.LOG").unwrap(),
            };

            let name = name.strip_suffix(".LOG");
            let name = match name {
                Some(v) => v,
                None => return heapless::String::from_str("00001.LOG").unwrap(),
            };

            let lognum = name.parse::<u16>();
            let lognum = match lognum {
                Ok(v) => v,
                Err(_) => return heapless::String::from_str("00001.LOG").unwrap(),
            };

            let mut s = heapless::String::new();
            core::fmt::write(
                &mut s,
                format_args!("{:0alignment$}.LOG", lognum + 1, alignment = 5),
            )
            .unwrap();
            return s;
        }

        async fn prepare_for_next_log<'a, IO, TP, OCC>(
            root_dir: &mut embedded_fatfs::Dir<'a, IO, TP, OCC>,
        ) -> Result<
            heapless::String<LOG_NAME_MAX_BYTES>,
            embedded_fatfs::Error<<IO as ErrorType>::Error>,
        >
        where
            IO: embedded_fatfs::ReadWriteSeek,
            TP: embedded_fatfs::TimeProvider,
            OCC: embedded_fatfs::OemCpConverter,
        {
            let mut last_log_txt = match root_dir.open_file("LAST_LOG.TXT").await {
                Ok(f) => f,
                Err(e) => match e {
                    embedded_fatfs::Error::NotFound => root_dir.create_file("LAST_LOG.TXT").await?,
                    _ => return Err(e),
                },
            };
            last_log_txt.seek(SeekFrom::Start(0)).await?;
            let mut buff = [0u8; LOG_NAME_MAX_BYTES];
            let logname = match last_log_txt.read_exact(&mut buff).await {
                Ok(_) => get_logname(&buff),
                Err(e) => match e {
                    embedded_io_async::ReadExactError::UnexpectedEof => get_logname(&buff[0..1]),
                    embedded_io_async::ReadExactError::Other(e) => return Err(e),
                },
            };
            last_log_txt.seek(SeekFrom::Start(0)).await?;
            last_log_txt.write(logname.as_bytes()).await?;
            last_log_txt.flush().await?;
            fmt::debug!("next log name {:?}", logname.as_str());
            Ok(logname.clone())
        }

        #[embassy_executor::task]
        pub async fn sdcard_task() {
            let p = unsafe { embassy_stm32::Peripherals::steal() };
            let mut sddriver = sdmmc::Sdmmc::new_4bit(
                p.SDMMC1,
                Irqs,
                p.PC12,
                p.PD2,
                p.PC8,
                p.PC9,
                p.PC10,
                p.PC11,
                Default::default(),
            );
            fmt::unwrap!(sddriver.init_card(time::mhz(24)).await);
            fmt::debug!("Configured clock: {}", sddriver.clock().0);

            #[cfg(feature = "sdmmc_perf_test")]
            {
                fmt::debug!("writing 25 MB");
                const MB25_IN_BLOCKS: usize = 25 * 1024 * 1024 /512;
                let mut block = sdmmc::DataBlock([0u8; 512]);
                block.fill(0xFF);
                let mut block_idx: u32 = 0;
                let tstart = embassy_time::Instant::now();
                let mut prev_mb = 0;
                for i in 0..MB25_IN_BLOCKS {
                    while let Err(e) = sddriver.write_block(block_idx, &block).await {
                        fmt::debug!("err {}", e);
                    }
                    block_idx += 1;
                    let kb_written = block_idx as u64 * 512/1024;
                    let mb_written = kb_written/1024;
                    if mb_written != prev_mb {
                        let tnow = embassy_time::Instant::now();
                        if let Some(rate) = kb_written.checked_div((tnow - tstart).as_secs()) {
                            fmt::debug!("written {}KB, rate {} KB/s", kb_written, rate);
                        }
                        prev_mb = mb_written;
                    }
                }
                let tend = embassy_time::Instant::now();
                fmt::debug!("writing done in: {}ms", (tend - tstart).as_millis());
                return;
            }

            let options = FsOptions::new();

            let sdcard = SdmmcInmemStorage::new(sddriver);
            let fs = fmt::unwrap!(FileSystem::new(sdcard, options).await);
            let mut root = fs.root_dir();
            let log_name = match prepare_for_next_log(&mut root).await {
                Ok(v) => v,
                Err(e) => {
                    fmt::debug!("preparing for next log failed {:?}", e);
                    panic!();
                }
            };

            let mut logf = fmt::unwrap!(root.create_file(&log_name).await);
            const BS_USIZE: usize = BLOCK_SIZE as usize;
            const MAX_BLOCKS_TO_READ: usize = 7;
            let mut read_buf = [0u8; BS_USIZE * MAX_BLOCKS_TO_READ];
            let mut bytes_written: usize = 0;
            let mut consumer = fmt::unwrap!(logger::get_consumer());
            logf.seek(SeekFrom::Start(0)).await.unwrap();
            logger::notify_driver_state(true);
            loop {
                let blocks_in_buf = consumer.len() / BS_USIZE;
                if blocks_in_buf == 0 {
                    embassy_futures::yield_now().await;
                    continue;
                }
                let bytes_to_pop = (blocks_in_buf * BS_USIZE).min(BS_USIZE * MAX_BLOCKS_TO_READ);
                let amt = consumer.pop_slice(&mut read_buf[..bytes_to_pop]);
                if amt == 0 {
                    embassy_futures::yield_now().await;
                    continue;
                }
                bytes_written = bytes_written.overflowing_add(amt).0;
                fmt::unwrap!(logf.write_all(&mut read_buf[..amt]).await);
                fmt::unwrap!(logf.flush().await);
                fmt::trace!("SD wrote {} bytes", bytes_written);
            }
        }
    }

    #[cfg(feature = "sdcard")]
    pub use sdcard::*;
}
