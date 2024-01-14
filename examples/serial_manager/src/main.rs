#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;
use embassy_executor::Spawner;
use embassy_time;

use bsp::{Board, GenericBoard, serial0_runner};
use serial_manager as serial;
mod fmt;

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
        let port = serial::find_by_protocol(serial::Protocol::MavlinkV2).unwrap();
        Self { port }
    }
}

fn serial_comm(_cts: &GlobalContext) {
    let port = &mut _cts.serial_task.borrow_mut().port;
    if port.available() == 0 {
        return;
    }
    while let Some(v) = port.read() {
        fmt::debug!("{}", v)
    }
    let s = "asd";
    port.write_slice(s.as_bytes()).unwrap();
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    Board::init();

    fmt::unwrap!(_spawner.spawn(serial0_runner(
        serial::Config::default().baud(115_200)
    )));

    let context = GlobalContext {
        serial_task: SerialTask1::new().into(),
    };

    let serial_task = scheduler::task!(serial_comm(&context));
    let tasks = [scheduler::Task::new(serial_task, 0.2, "task1")];

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
mod nondefmt {
    use core::panic::PanicInfo;
    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        loop {}
    }
}
