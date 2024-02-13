#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;

use bsp::*;
mod fmt;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::task]
async fn task1() {
    loop {
        fmt::info!("Hello from task!");
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    Board::init();
    fmt::unwrap!(_spawner.spawn(sdcard_task()));
    // fmt::unwrap!(_spawner.spawn(task1()));

    // let msg = logger::LogMessages::LogImuRaw(logger::ImuRaw {
    //     sample_time: embassy_time::Instant::now().as_micros(),
    //     acc: [0.0, 0.0, 9.81],
    //     gyr: [0.0, 0.0, 0.0],
    // });
    //
    // logger::log_msg(msg);
    //
    // loop {
    //     for _ in 0..512 / 44 {
    //         let msg = logger::LogMessages::LogINS(logger::INS {
    //             sample_time: embassy_time::Instant::now().as_micros(),
    //             att: [1.0, 2.0, 3.0],
    //             acc: [0.0, 0.0, 9.81],
    //             gyr: [0.0, 0.0, 0.0],
    //         });
    //         logger::log_msg(msg);
    //     }
    //     embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    // }

    // fmt::info!("writing success");
}

#[cfg(not(feature = "std"))]
#[cfg(feature = "defmt")]
defmt::timestamp!("{=u32:us}", {
    embassy_time::Instant::now().as_micros() as u32
});

#[cfg(not(feature = "std"))]
#[cfg(not(feature = "defmt"))]
mod nondefmt {
    use core::panic::PanicInfo;
    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        loop {}
    }
}
