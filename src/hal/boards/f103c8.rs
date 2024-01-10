use cortex_m::interrupt;
use embassy_stm32::{bind_interrupts, peripherals, time, usart, Config};

use crate::libs::fmt;

use super::gpio;
use super::serial::*;
use super::GpioOutput;

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
        gpio::bind_gpio(2, init_led_1);
        let sc = static_cell::make_static!(ringbuf::StaticRb::<u8, 512>::default());
        let (prod, con) = sc.split_ref();
        serial_manager::bind_port(serial_manager::Protocol::Test, __t2(), con, prod);

        unsafe { interrupt::enable() };
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

fn __t2() -> super::RunnerFType {
    serial1_runner
}

#[embassy_executor::task]
async fn serial1_runner(cfg: serial_manager::Config, mut rb: ringbuf::StaticProducer<'static, u8, 512>) {
    let p = unsafe { embassy_stm32::Peripherals::steal() };
    let uc: usart::Config = cfg.into();
    let mut port = usart::Uart::new(p.USART1, p.PA10, p.PA9, Irqs, p.DMA1_CH4, p.DMA1_CH5, uc).unwrap();
    let mut buf = [0u8; 100];
    loop {
        let res = port.read_until_idle(&mut buf).await.map_err(|e| fmt::debug!("read err {}", e));
        match res {
            Ok(v) => {
                rb.push_slice(&buf[..v]);
            },
            Err(_) => continue,
        }
    }
}

pub fn get_input_pin(
) -> impl embedded_hal_async::digital::Wait + embedded_hal::digital::v2::InputPin {
    let p = unsafe { embassy_stm32::Peripherals::steal() };
    embassy_stm32::exti::ExtiInput::new(
        embassy_stm32::gpio::Input::new(p.PA15, embassy_stm32::gpio::Pull::Up),
        p.EXTI15,
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
