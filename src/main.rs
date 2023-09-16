#![cfg_attr(not(feature="std"), no_std)]
#![cfg_attr(not(feature="std"), no_main)]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

mod boards;
use boards::GenericBoard;

mod fmt;

#[cfg(feature="defmt")]
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::task]
async fn task1() {
    info!("Hello from task!");
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    boards::Board::init();
    unwrap!(_spawner.spawn(task1()));

    loop {
        info!("Hello from main!");
        Timer::after(Duration::from_secs(1)).await;
    }
}
