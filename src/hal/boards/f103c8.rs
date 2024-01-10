use crate::hal::serial;
use crate::libs::fmt;
use cortex_m::interrupt;
use embassy_stm32::{bind_interrupts, peripherals, time, usart, Config};

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

fn serial1_bind() {
    let sc: &'static mut serial::SerialRingBuf =
        static_cell::make_static!(ringbuf::StaticRb::default());
    let (prod, con) = sc.split_ref();
    serial::bind_port(serial::Protocol::Test, 1, con, prod);
}

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

pub mod hw_tasks {
    use super::*;

    #[embassy_executor::task]
    pub async fn serial1_runner(cfg: serial::Config) {
        let uc: usart::Config = cfg.into();

        let p = unsafe { embassy_stm32::Peripherals::steal() };
        let mut port = fmt::unwrap!(usart::Uart::new(
            p.USART1, p.PA10, p.PA9, Irqs, p.DMA1_CH4, p.DMA1_CH5, uc
        ));
        let mut rb = fmt::unwrap!(serial::find_write_ref(1));

        let mut buf = [0u8; 100];

        loop {
            let res = port
                .read_until_idle(&mut buf)
                .await
                .map_err(|e| fmt::debug!("read err {}", e));
            match res {
                Ok(v) => {
                    rb.push_slice(&buf[..v]);
                }
                Err(_) => continue,
            }
        }
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
