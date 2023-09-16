
impl super::GenericBoard for super::Board {
    fn init() {
        let mut config = embassy_stm32::Config::default();
        config.rcc.sys_ck = Some(embassy_stm32::time::Hertz(84_000_000));
        let _p = embassy_stm32::init(config);
    }
}

#[cfg(not(feature="defmt"))]
mod nondefmt {
    use core::panic::PanicInfo;
    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        loop {}
    }
}

