#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;

use bsp::*;
mod fmt;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};


#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    Board::init();

    loop {
        fmt::info!("Hello World!");
        embassy_time::Timer::after_secs(1).await;
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
