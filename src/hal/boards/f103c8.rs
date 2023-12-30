use embassy_stm32::{time, Config};

#[path = "../serial/dummy_serial.rs"]
mod dummy_serial;

impl super::GenericBoard for super::Board {
    fn init() {
        let mut config = Config::default();
        config.rcc.sys_ck = Some(time::mhz(72));
        config.rcc.hse = Some(time::mhz(8));
        let _ = embassy_stm32::init(config);
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
