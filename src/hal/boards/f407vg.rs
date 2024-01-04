use embassy_stm32::{bind_interrupts, peripherals, time, usart, Config};

use super::*;
use crate::libs::fmt;

use static_cell::make_static;

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    USART3 => usart::InterruptHandler<peripherals::USART3>;
});


// struct SerialNoDma<'lt, U: usart::BasicInstance>(usart::Uart<'lt, U>);
//
// impl<'lt, U: usart::BasicInstance> GenericSerialTrait for SerialNoDma<'lt, U> {
//     async fn read(&mut self, _buf: &mut [u8]) -> Result<usize, ()> {
//         todo!();
//     }
//     async fn write(&mut self, _buf: &[u8]) -> Result<usize, ()> {
//         todo!();
//     }
//     fn split(self) -> (impl embedded_io_async::Write, impl embedded_io_async::Read) {
//         todo!()
//     }
// }
//

struct SerialDma<'lt, U: usart::BasicInstance, DT, DR> {
    port: usart::Uart<'lt, U, DT, DR>,
    name: &'lt str,
    rx_buf: &'lt mut [u8],
}

impl<'lt, U: usart::BasicInstance, DT: usart::TxDma<U>, DR: usart::RxDma<U>> GenericSerialTrait
    for SerialDma<'lt, U, DT, DR>
{
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
        self.port
            .read_until_idle(buf)
            .await
            .map_err(|e| fmt::debug!("{} read err: {}", self.name, e))
    }

    async fn write(&mut self, buf: &[u8]) -> Result<usize, ()> {
        self.port
            .write(buf)
            .await
            .map_err(|e| fmt::debug!("{} write err: {}", self.name, e))?;
        Ok(buf.len())
    }

    fn split(
        self,
    ) -> (
        impl embedded_io_async::Write + 'lt,
        impl embedded_io_async::Read + 'lt,
    ) {
        let (tx, rx) = self.port.split();
        let rx = rx.into_ring_buffered(self.rx_buf);
        (tx, rx)
    }
}

impl GenericBoard for Board {
    fn init() {
        let mut config = Config::default();
        config.rcc.sys_ck = Some(time::Hertz(84_000_000));
        let _ = embassy_stm32::init(config);
        unsafe {interrupt::enable()};
        serial_manager::bind_port(serial_manager::Protocol::Test, init_serial1);
    }
}

fn init_serial1(config: serial_manager::Config) -> GenericSerial {
    let p = unsafe { embassy_stm32::Peripherals::steal() };
    let uc: usart::Config = config.into();
    let port = usart::Uart::new(p.USART2, p.PA3, p.PA2, Irqs, p.DMA1_CH6, p.DMA1_CH5, uc);
    let rx_buf = make_static!([0u8; 256]);

    SerialDma {
        port,
        name: "serial1",
        rx_buf,
    }
}

#[cfg(not(feature = "defmt"))]
mod nondefmt {
    use core::panic::PanicInfo;
    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        loop {}
    }
}
