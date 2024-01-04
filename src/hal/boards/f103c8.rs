use cortex_m::interrupt;
use embassy_stm32::{time, Config};
use super::GpioOutput;
use super::gpio;


#[path = "../serial/dummy_serial.rs"]
mod dummy_serial;

impl super::GenericBoard for super::Board {
    fn init() {
        let mut config = Config::default();
        config.rcc.hclk = Some(time::mhz(72));
        config.rcc.sys_ck = Some(time::mhz(72));
        config.rcc.hse = Some(time::mhz(8));
        config.rcc.pclk1 = Some(time::mhz(36));
        config.rcc.pclk2 = Some(time::mhz(72));
        #[cfg(feature="defmt")] {
            config.enable_debug_during_sleep = true;
        }
        #[cfg(not(feature="defmt"))] {
            config.enable_debug_during_sleep = false;
        }

        let _ = embassy_stm32::init(config);
        gpio::bind_gpio(2, init_led_1);
        unsafe {interrupt::enable()};
    }
}

fn init_led_1(_cfg: gpio::Config) -> GpioOutput {
        let p = unsafe { embassy_stm32::Peripherals::steal() };
        embassy_stm32::gpio::Output::new(
            p.PC13,
            embassy_stm32::gpio::Level::Low,
            embassy_stm32::gpio::Speed::Low,
        )
}

#[cfg(not(feature = "defmt"))]
mod nondefmt {
    use core::panic::PanicInfo;
    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        loop {}
    }
}
