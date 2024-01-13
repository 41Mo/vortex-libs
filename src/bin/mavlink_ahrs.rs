#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;
use embassy_executor::Spawner;
use embassy_time;

#[path = "../bsp/mod.rs"]
mod bsp;
#[path = "../libs/mod.rs"]
mod libs;
use bsp::{Board, GenericBoard, boards};
use libs::*;
use serial_manager as serial;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

pub struct GlobalContext {
    ins: RefCell<ins::INS>,
    gcs: RefCell<gcs::GcsMavlink>,
}


fn serial_comm(_cts: &GlobalContext) {
    let gcs = &mut _cts.gcs.borrow_mut();

    gcs.update_recieve();
    gcs.update_send(&_cts.ins.borrow());
}

fn ins_update(_cts: &GlobalContext) {
    _cts.ins.borrow_mut().update()
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    Board::init();

    // fmt::unwrap!(_spawner.spawn(hal::boards::serial1_runner(
    //     hal::serial::Config::default().baud(115_200)
    // )));
    fmt::unwrap!(_spawner.spawn(boards::serial0_runner(serial::Config::default())));

    let ringbuf = static_cell::make_static!(ringbuf::StaticRb::<
        (nalgebra::Vector3<f32>, nalgebra::Vector3<f32>),
        5,
    >::default());
    let (prod, con) = ringbuf.split_ref();
    fmt::unwrap!(_spawner.spawn(boards::imu_task(prod)));

    let context = GlobalContext {
        gcs: gcs::GcsMavlink::new().into(),
        ins: ins::INS::new(con).into(),
    };

    let serial_task = scheduler::task!(serial_comm(&context));
    let ins_task = scheduler::task!(ins_update(&context));
    let tasks = [
        scheduler::Task::new(serial_task, 50.0, "task1"),
        scheduler::Task::new(ins_task, 100.0, "ins"),
    ];

    const SCHED_LOOP_RATE_HZ: u32 = 400;
    // this is a syncronous scheduler
    let mut s = scheduler::Scheduler::new(tasks, SCHED_LOOP_RATE_HZ);

    loop {
        s.update();
        embassy_time::Timer::after(embassy_time::Duration::from_hz(SCHED_LOOP_RATE_HZ.into()))
            .await;
    }
}
