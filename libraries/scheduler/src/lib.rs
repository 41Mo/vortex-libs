#![no_std]
use core::ops::*;

pub struct Task<T: Fn()> {
    function: T,
    name: &'static str,
    rate_hz: f32,
    max_time_micros: u16,
    priority: u8,
}

impl<T: Fn()> Task<T> {
    pub fn new(function: T, rate_hz: f32, name: &'static str) -> Self {
        Task {
            function,
            rate_hz,
            name,
            max_time_micros: 0,
            priority: 0,
        }
    }
}

pub struct Scheduler<const T: usize, F: Fn()> {
    tasks: [Task<F>; T],
    last_run: [u16; T],
    tick_counter: u16,
    loop_rate_hz: u32,
    loop_period_us: u32,
    task_time_allowed: u32,
    task_time_started: u32,
}

impl<const T: usize, F:Fn()> Scheduler<T, F> {
    pub fn new(tasks: [Task<F>; T], loop_rate_hz: u32) -> Self {
        Scheduler {
            tasks,
            last_run: [0; T],
            tick_counter: 0,
            loop_period_us: 1000000u32 / loop_rate_hz,
            task_time_started: 0,
            task_time_allowed: 0,
            loop_rate_hz,
        }
    }

    fn tick(&mut self) {
        self.tick_counter = self.tick_counter.overflowing_add(1).0;
    }

    pub fn update(&mut self) {
        self.tick();
        self.run();
    }

    fn run(&mut self) {
        for (i, task) in self.tasks.iter_mut().enumerate() {
            self.task_time_allowed = self.loop_period_us;
            let dt = self.tick_counter.overflowing_sub(self.last_run[i]).0;
            let interval_ticks: u32 = (self.loop_rate_hz as f32 / task.rate_hz) as u32;
            if (dt as u32) < interval_ticks {
                // this task is not yet scheduled to run again
                continue;
            }
            (task.function)();
            self.last_run[i] = self.tick_counter;
        }
    }
}

#[macro_export]
macro_rules! task {
    ($fn:expr) => {
        &(|| {$fn}) as &dyn Fn()
    }
}

// #[allow(unused)]
// pub use task;

#[cfg(test)]
mod tests {
    use super::*;

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
        // this module should be accesed only in single thread
        pub struct INS {
            roll: f32,
            pitch: f32,
        }

        impl INS {
            pub const fn new() -> Self {
                Self {
                    roll: 0f32,
                    pitch: 0f32,
                }
            }

            pub fn get_pitch_cd(&self) -> i32 {
                (self.pitch * 100.0) as i32
            }
        }

        pub fn update(_ctx: &super::context::GlobalContext) {
            println!("INS update");
            let mut ins = _ctx.ins.borrow_mut();
            ins.pitch += 1f32;
        }
    }

    mod gps {
        pub struct GPS {
            lat: u32,
            lng: u32,
        }

        impl GPS {
            pub const fn new() -> Self {
                Self { lat: 0, lng: 0 }
            }

            pub fn gps_get_lat(&self) -> u32 {
                self.lat
            }
        }

        pub fn update(_ctx: &super::context::GlobalContext) {
            println!("GPS update");
            let ins = _ctx.ins.borrow();
            println!("ins pitch {}", ins.get_pitch_cd());
        }

    }

    const SCHED_LOOP_RATE_HZ: u32 = 100;

    #[test]
    fn init() {
        let context = context::GlobalContext::new();
        let a = task!(ins::update(&context));
        let b = task!(gps::update(&context));
        let tasks = [
            Task::new(a, 5.0, "task1"),
            Task::new(b, 5.0, "task2"),
        ];

        let mut s = Scheduler::new(tasks, SCHED_LOOP_RATE_HZ);
        loop {
            s.update();
            std::thread::sleep(std::time::Duration::from_millis(
                (1000 / SCHED_LOOP_RATE_HZ).into(),
            ))
        }
    }
}
