#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]
#![feature(type_alias_impl_trait)]

use bsp::{imu_task, sdcard_task, serial0_runner, Board, GenericBoard};
use serial_manager as serial;

mod fmt;
mod tasks;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

static EXECUTOR_LOW: static_cell::StaticCell<embassy_executor::Executor> =
    static_cell::StaticCell::new();

#[cortex_m_rt::entry]
fn main() -> ! {
    let (executor_high, executor_med) = Board::init();

    let executor_low = EXECUTOR_LOW.init(embassy_executor::Executor::new());


    fmt::unwrap!(executor_med.spawn(tasks::start_tasks()));
    fmt::unwrap!(executor_high.spawn(imu_task()));

    // io tasks are low prio tasks
    executor_low.run(|s| {
        fmt::unwrap!(s.spawn(sdcard_task()));
        fmt::unwrap!(s.spawn(serial0_runner(serial::Config::default())));
    });
}

#[cfg(not(feature = "std"))]
#[cfg(feature = "defmt")]
defmt::timestamp!("{=u32:us}", {
    embassy_time::Instant::now().as_micros() as u32
});

#[cfg(not(feature = "defmt"))]
mod nondefmt {
    use core::panic::PanicInfo;
    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        loop {}
    }
}
