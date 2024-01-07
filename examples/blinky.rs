#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time;
use embedded_hal::digital::v2::OutputPin;

#[path = "../src/hal/mod.rs"]
mod hal;
#[path = "../src/libs/mod.rs"]
mod libs;
use hal::*;
use libs::*;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    Board::init();
    let led = gpio::find_by_number(2, gpio::Config::default());
    let mut led = match led {
        Some(l) => l,
        None => {
            fmt::error!("board doesn't have led1 defined");
            return;
        }
    };
    loop {
        let _ = led.set_high();
        embassy_time::Timer::after(embassy_time::Duration::from_millis(300)).await;
        let _ = led.set_low();
        embassy_time::Timer::after(embassy_time::Duration::from_millis(300)).await;
    }
}
