#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![feature(return_position_impl_trait_in_trait)]

use embassy_executor::Spawner;

#[path = "../hal/mod.rs"]
mod hal;
#[path = "../libs/mod.rs"]
mod libs;
use hal::*;
use libs::*;


#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::task]
async fn task1() {
    fmt::info!("Hello from task!");
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    Board::init();
    fmt::unwrap!(_spawner.spawn(task1()));

}
