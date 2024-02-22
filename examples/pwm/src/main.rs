#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;

use bsp::*;
mod fmt;
mod tasks;
use serial_manager as serial;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    Board::init();
    serial_manager::bind_ports(&[
        (0, 0),
    ]);
    {
        use servo_control::*;
        setup_channel(
            ServoChannel::new()
                .set_servo_channel(ChNum::Ch3)
                .set_function(ControlSurface::Aileron)
                .set_trim_period(1500),
        );
        setup_channel(
            ServoChannel::new()
                .set_servo_channel(ChNum::Ch4)
                .set_function(ControlSurface::Aileron)
                .set_trim_period(1500),
        );

        control_surface_onoff(ControlSurface::Aileron, true);
    }

    fmt::unwrap!(_spawner.spawn(serial_runner(0, serial::Config::default())));
    fmt::unwrap!(_spawner.spawn(bsp::pwm_task()));
    tasks::start_tasks().await;
}

#[cfg(not(feature = "std"))]
#[cfg(not(feature = "defmt"))]
mod nondefmt {
    use core::panic::PanicInfo;
    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        loop {}
    }
}
