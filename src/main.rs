#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::task]
async fn task1() {
    info!("Hello from task!");
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut config = Config::default();
    config.rcc.sys_ck = Some(Hertz(84_000_000));
    let _p = embassy_stm32::init(config);

    defmt::unwrap!(_spawner.spawn(task1()));

    loop {
        info!("Hello from main!");
        Timer::after(Duration::from_secs(1)).await;
    }
}
