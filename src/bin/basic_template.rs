#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]
#![feature(type_alias_impl_trait)]
#![feature(generic_arg_infer)]
#![feature(const_mut_refs)]

use embassy_executor::Spawner;

#[path = "../hal/mod.rs"]
mod hal;
#[path = "../libs/mod.rs"]
mod libs;

use hal::*;
use libs::*;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};


#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    Board::init();
}
