#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;

use embassy_executor::{Spawner, SpawnToken};
use embassy_time;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;

#[path = "../hal/mod.rs"]
mod hal;
#[path = "../libs/mod.rs"]
mod libs;
use hal::f103c8::get_input_pin;
use hal::{serial::serial_manager::SerialPort, *};
use libs::*;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

pub struct GlobalContext {
    SerialCommunicator1: RefCell<SerialCommunicator1>,
}

struct SerialCommunicator1 {
    port: SerialPort,
}

impl SerialCommunicator1 {
    fn new() -> Self {
        let port = serial::serial_manager::find_by_protocol(serial::serial_manager::Protocol::Test)
            .unwrap();
        Self { port }
    }
}

fn serial_read(_cts: &GlobalContext) {
    let serial = &mut _cts.SerialCommunicator1.borrow_mut().port;
    if serial.len() == 0 {
        return;
    }

    while let Some(v) = serial.pop() {
        fmt::debug!("{}", v)
    }
}

fn printer() {
    fmt::debug!("printing")
}

#[embassy_executor::task]
async fn asd() {
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    Board::init();
    // let mut pin = get_input_pin();
    let context = GlobalContext {
        SerialCommunicator1: SerialCommunicator1::new().into(),
    };

    let (task, producer) =
        serial::serial_manager::find_port_runner(serial::serial_manager::Protocol::Test).unwrap();

    fmt::unwrap!(_spawner.spawn(task(
        serial::serial_manager::Config::default().baud(115200),
        producer
    )));

    let t = asd();

    let serial_task = &(|| serial_read(&context)) as &dyn Fn();
    let tasks = [
        scheduler::Task::new(serial_task, 0.2, "task1"),
        // scheduler::Task::new(&printer, 1.0, "task2"),
    ];

    const SCHED_LOOP_RATE_HZ: u32 = 100;
    // this is a syncronous scheduler
    let mut s = scheduler::Scheduler::new(tasks, SCHED_LOOP_RATE_HZ);

    loop {
        s.update();
        embassy_time::Timer::after(embassy_time::Duration::from_hz(SCHED_LOOP_RATE_HZ.into()))
            .await;
    }
}
