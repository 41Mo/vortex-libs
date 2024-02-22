#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;
use embassy_executor::Spawner;
use embassy_time;

use bsp::{serial_runner, Board, GenericBoard};
use serial_manager as serial;
mod fmt;

#[repr(u8)]
enum SerialProtocols {
    Text,
}

#[repr(u8)]
enum SerialPortName {
    Serial0,
    Serial1,
}

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

pub struct GlobalContext {
    serial_task: RefCell<SerialTask1>,
}

struct SerialTask1 {
    port: serial::SerialPort,
}

impl SerialTask1 {
    fn new() -> Self {
        let port = serial::find_by_protocol(SerialProtocols::Text as u8).unwrap();
        Self { port }
    }
}

fn serial_comm(_cts: &GlobalContext) {
    fmt::debug!("task");
    let port = &mut _cts.serial_task.borrow_mut().port;
    while let Some(v) = port.read() {
        fmt::debug!("{}", v)
    }
    let s = "asd";
    port.write_slice(s.as_bytes()).unwrap();
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    Board::init();
    serial_manager::bind_ports(&[
        (SerialPortName::Serial0 as u8, SerialProtocols::Text as u8),
        (SerialPortName::Serial1 as u8, SerialProtocols::Text as u8),
    ]);

    #[cfg(not(feature = "std"))]
    {
        let mut cfg = serial::Config::default();
        cfg.baud = 115_200;
        fmt::unwrap!(_spawner.spawn(serial_runner(SerialPortName::Serial0 as u8, cfg)));
    }

    #[cfg(feature = "std")]
    {
        use std::str::FromStr;
        for arg in std::env::args() {
            fmt::debug!("arg: {}", arg);
        }
        let mut cfg = serial_manager::Config::default();
        cfg.dev = heapless::String::from_str("127.0.0.1:11210")
            .expect("Unable to make heapless string from str");
        fmt::unwrap!(_spawner.spawn(serial_runner(SerialPortName::Serial0 as u8, cfg)));
    }

    let context = GlobalContext {
        serial_task: SerialTask1::new().into(),
    };
    let serial_task = scheduler::task!(serial_comm(&context));
    let tasks = [scheduler::Task::new(serial_task, 1.0, "task1")];
    const SCHED_LOOP_RATE_HZ: u32 = 100;
    // this is a syncronous scheduler
    let mut s = scheduler::Scheduler::new(tasks, SCHED_LOOP_RATE_HZ);

    loop {
        s.update();
        embassy_time::Timer::after(embassy_time::Duration::from_hz(SCHED_LOOP_RATE_HZ.into()))
            .await;
    }
}

#[cfg(not(feature = "defmt"))]
#[cfg(not(feature = "std"))]
mod nondefmt {
    use core::panic::PanicInfo;
    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        loop {}
    }
}
