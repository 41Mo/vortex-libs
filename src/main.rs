#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![feature(return_position_impl_trait_in_trait)]

use embassy_executor::Spawner;
use embassy_futures::join::join;

mod hal;
use crate::hal::boards::GenericBoard;
use crate::hal::boards::GenericSerialTrait;
use crate::hal::serial_manager;
use crate::hal::Board;
use embedded_io_async::{Read, Write};

mod fmt;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::task]
async fn task1() {
    info!("Hello from task!");
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    Board::init();
    unwrap!(_spawner.spawn(task1()));

    /*
     * unsplitted transfer
     */
    // let transfer = async move {
    //     let mut buf = [0u8; 1024];
    //     let uart = serial_manager::find_by_protocol(serial_manager::Protocol::Test)
    //         .expect("unable to find port");
    //     let mut u: crate::hal::boards::GenericSerial =
    //         uart.begin(serial_manager::Config::default().baud(115_200));
    //
    //     let mut now = embassy_time::Instant::now();
    //     loop {
    //         if now.elapsed().as_secs() >= 1 {
    //             u.write(b"hello").await.unwrap();
    //             now = embassy_time::Instant::now();
    //         }
    //         if let Ok(v) = embassy_time::with_timeout(
    //             embassy_time::Duration::from_millis(100),
    //             u.read(&mut buf),
    //         )
    //         .await
    //         {
    //             match v {
    //                 Ok(v) => info!("recv: {:?}", &buf[..v]),
    //                 Err(e) => error!("recv err {:?}", e),
    //             }
    //         }
    //     }
    // };

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
                    Ok(v) => info!("recieved: {:#?}", &buf[..v]),
                    Err(_) => debug!("read error"),
                }
            }
        };

        let write_fut = async move {
            loop {
                if let Err(_) = tx.write_all(b"hello\n").await {
                    error!("write error")
                }
                embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await
            }
        };

        join(read_fut, write_fut).await;
    };

    let hello = async move {
        info!("Hello from main!");
    };

    join(hello, transfer).await;
}
