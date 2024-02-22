#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time;

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

mod context {
    use core::cell::RefCell;
    pub struct GlobalContext {
        pub ins: RefCell<super::ins::INS>,
        pub gps: RefCell<super::gps::GPS>,
    }

    impl GlobalContext {
        pub fn new() -> Self {
            Self {
                ins: super::ins::INS::new().into(),
                gps: super::gps::GPS::new().into(),
            }
        }
    }
}

mod ins {
    use super::fmt;
    // this module should be accesed only in single thread
    pub struct INS {
        pitch: f32,
    }

    impl INS {
        pub const fn new() -> Self {
            Self {
                pitch: 0f32,
            }
        }

        pub fn get_pitch_cd(&self) -> i32 {
            (self.pitch * 100.0) as i32
        }
    }

    pub fn update(_ctx: &super::context::GlobalContext) {
        fmt::info!("INS update");
        let mut ins = _ctx.ins.borrow_mut();
        ins.pitch += 1f32;
    }
}

mod gps {
    use super::fmt;
    pub struct GPS;
    impl GPS {
        pub const fn new() -> Self {
            Self {}
        }
    }

    pub fn update(_ctx: &super::context::GlobalContext) {
        fmt::info!("GPS update");
        let ins = _ctx.ins.borrow();
        fmt::info!("ins pitch {}", ins.get_pitch_cd());
    }
}


#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    Board::init();
    fmt::unwrap!(_spawner.spawn(task1()));
    let context = context::GlobalContext::new();
    let ins_task = &(|| ins::update(&context)) as &dyn Fn();
    let gps_task = &(|| gps::update(&context)) as &dyn Fn();
    let tasks = [
        scheduler::Task::new(ins_task, 1.0, "task1"),
        scheduler::Task::new(gps_task, 1.0, "task2"),
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

#[cfg(not(feature = "defmt"))]
#[cfg(not(feature = "std"))]
mod nondefmt {
    use core::panic::PanicInfo;
    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        loop {}
    }
}
