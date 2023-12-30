#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![feature(return_position_impl_trait_in_trait)]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embedded_io_async::{Read,Write};

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
    /*
     * splitted transfer
     */
    let transfer = async move {
        let mut buf = [0u8; 1024];
        let uart = serial_manager::find_by_protocol(serial_manager::Protocol::Test)
            .expect("unable to find port");
        let u: crate::hal::boards::GenericSerial =
            uart.begin(serial_manager::Config::default().baud(115_200));

        let (tx, rx) = u.split();
        let (tx, rx) = &mut (tx, rx);

        let read_fut = async move {
            loop {
                match rx.read(&mut buf).await {
                    Ok(v) => fmt::info!("recieved: {:#?}", &buf[..v]),
                    Err(_) => fmt::debug!("read error"),
                }
            }
        };

        let write_fut = async move {
            loop {
                if let Err(_) = tx.write_all(b"hello\n").await {
                    fmt::error!("write error")
                }
                embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await
            }
        };

        join(read_fut, write_fut).await;
    };

    transfer.await;
}
