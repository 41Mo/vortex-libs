#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;

use bsp::{imu_task, serial0_runner, Board, GenericBoard};
use serial_manager as serial;

mod fmt;
mod tasks;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    Board::init();

    fmt::unwrap!(_spawner.spawn(serial0_runner(serial::Config::default())));
    fmt::unwrap!(_spawner.spawn(imu_task()));
    tasks::start_tasks().await;
}

#[cfg(not(feature = "defmt"))]
mod nondefmt {
    use core::panic::PanicInfo;
    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        loop {}
    }
}
