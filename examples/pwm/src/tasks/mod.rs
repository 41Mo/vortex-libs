// Safety:
// this module tries to guarantee, that only single threaded
// access would be to submodules
// thats why all modules inside should be singlethreaded inside scheduler

mod gcs;

pub(crate) async fn start_tasks() -> ! {
    let serial_tx = scheduler::task!(gcs::update_send());
    let serial_rx = scheduler::task!(gcs::update_recieve());
    let tasks = [
        scheduler::Task::new(serial_rx, 50.0, "serial_rx"),
        scheduler::Task::new(serial_tx, 50.0, "serial_tx"),
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
