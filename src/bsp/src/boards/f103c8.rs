use serial_manager as serial;
use crate::fmt;
use cortex_m::interrupt;
use embassy_stm32::{bind_interrupts, peripherals, time, usart, Config};

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

impl super::GenericBoard for super::Board {
    fn init() {
        let mut config = Config::default();
        config.rcc.hclk = Some(time::mhz(72));
        config.rcc.sys_ck = Some(time::mhz(72));
        config.rcc.hse = Some(time::mhz(8));
        config.rcc.pclk1 = Some(time::mhz(36));
        config.rcc.pclk2 = Some(time::mhz(72));
        #[cfg(feature = "defmt")]
        {
            config.enable_debug_during_sleep = true;
        }
        #[cfg(not(feature = "defmt"))]
        {
            config.enable_debug_during_sleep = false;
        }

        let _ = embassy_stm32::init(config);

        serial1_bind();

        unsafe { interrupt::enable() };
    }
}

fn serial1_bind() {
    let sc1: &'static mut serial::SerialRingBuf =
        static_cell::make_static!(ringbuf::StaticRb::default());

    let sc2: &'static mut serial::SerialRingBuf =
        static_cell::make_static!(ringbuf::StaticRb::default());
    let (p1, c1) = sc1.split_ref();
    let (p2, c2) = sc2.split_ref();
    serial::bind_port(serial::Protocol::MavlinkV2, 1, c1, p1, c2, p2);
}

pub mod hw_tasks {
    use embedded_io_async::Write;

    use super::*;

    #[allow(unused)]
    async fn port_read<T: usart::BasicInstance, R: usart::RxDma<T>>(
        mut port: usart::UartRx<'_, T, R>,
        mut rb_producer: serial::RingBufWriteRef,
        name: &str,
    ) {
        let mut buf = [0u8; 100];
        loop {
            let res = port
                .read_until_idle(&mut buf)
                .await
                .map_err(|e| fmt::debug!("{}, read err {}", name, e));
            match res {
                Ok(v) => {
                    let amt = core::cmp::min(rb_producer.free_len(), v);
                    rb_producer.push_slice(&buf[..amt]);
                    if amt < v {
                        fmt::debug!("{}, RBfull. written {}, requested {}", name, amt, v)
                    }
                }
                Err(_) => continue,
            }
        }
    }

    #[allow(unused)]
    async fn port_write<T: usart::BasicInstance, R: usart::TxDma<T>>(
        mut port: usart::UartTx<'_, T, R>,
        mut consumer: serial::RingBufReadRef,
        name: &str,
    ) {
        let mut buf = [0u8; 100];
        loop {
            if consumer.len() == 0 {
                embassy_time::Timer::after_micros(5).await;
            }

            let amt = consumer.pop_slice(&mut buf);
            let _ = port.write_all(&buf[..amt])
                .await
                .map_err(|e| fmt::debug!("{}, write error {}", name, e));
        }
    }

    #[embassy_executor::task]
    pub async fn serial1_runner(cfg: serial::Config) {
        let mut uc = usart::Config::default();
        uc.baudrate = cfg.baud;

        let p = unsafe { embassy_stm32::Peripherals::steal() };
        let port = fmt::unwrap!(usart::Uart::new(
            p.USART1, p.PA10, p.PA9, Irqs, p.DMA1_CH4, p.DMA1_CH5, uc
        ));
        let (consumer, producer) = fmt::unwrap!(serial::find_port_rb_ref(1));

        let (tx, rx) = port.split();
        let name = "Serial1";

        let reader = port_read(rx, producer, name);
        let writer = port_write(tx, consumer, name);

        embassy_futures::join::join(reader, writer).await;
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
