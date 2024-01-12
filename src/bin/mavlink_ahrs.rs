#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;
use embassy_executor::Spawner;
use embassy_time;
use mavlink;

#[path = "../hal/mod.rs"]
mod hal;
#[path = "../libs/mod.rs"]
mod libs;
use hal::{serial, Board, GenericBoard};
use libs::*;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

pub struct GlobalContext {
    ins: RefCell<INS>,
    gcs: RefCell<GcsMavlink>,
}

struct INS {
    imu: ringbuf::StaticConsumer<'static, (nalgebra::Vector3<f32>, nalgebra::Vector3<f32>), 5>,
    dcm: nalgebra::Matrix3<f32>,
    rad_corr_c: f32,
}

impl INS {
    fn new(
        rb: ringbuf::StaticConsumer<'static, (nalgebra::Vector3<f32>, nalgebra::Vector3<f32>), 5>,
    ) -> Self {
        Self {
            imu: rb,
            dcm: nalgebra::Matrix3::<f32>::from_diagonal_element(1.0),
            rad_corr_c: 1.0,
        }
    }

    fn get_rpy(&self) -> (f32, f32, f32) {
        get_angles(&self.dcm)
    }

    fn update(&mut self) {
        if self.imu.len() == 0 {
            return;
        }
        let (acc, gyr) = self.imu.pop().unwrap();
        let dt = 0.01;
        let a_enu = self.dcm * acc;
        let mut w_enu = nalgebra::Vector3::<f32>::zeros();
        w_enu[0] = a_enu[1] * -self.rad_corr_c;
        w_enu[1] = a_enu[0] * self.rad_corr_c;
        let w_enu_hat = nalgebra::Matrix3::<f32>::new(
            0.0, -w_enu[2], w_enu[1], w_enu[2], 0.0, -w_enu[0], -w_enu[1], w_enu[0], 0.0,
        );
        let w_body_hat = nalgebra::Matrix3::<f32>::new(
            0.0, -gyr[2], gyr[1], gyr[2], 0.0, -gyr[0], -gyr[1], gyr[0], 0.0,
        );
        self.dcm += (self.dcm * w_body_hat - w_enu_hat * self.dcm) * dt;
    }
}

struct GcsMavlink {
    port: serial::SerialPort,
    mav_header: mavlink::MavHeader,
    last_update_time: [embassy_time::Instant; 2],
    message_rates: [(MSGID, u32); 2],
}

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum MSGID {
    HEARTBEAT = 1,
    ATTITUDE = 30,
}

impl Into<u32> for MSGID {
    fn into(self) -> u32 {
        self as u32
    }
}


impl GcsMavlink {
    fn new() -> Self {
        let port = serial::find_by_protocol(serial::Protocol::Test).unwrap();
        Self {
            port,
            mav_header: mavlink::MavHeader {
                system_id: 1,
                component_id: 1,
                sequence: 0,
            },
            last_update_time: [embassy_time::Instant::from_secs(0); 2],
            message_rates: [(MSGID::HEARTBEAT, 1), (MSGID::ATTITUDE, 0)],
        }
    }

    fn send_message<M: mavlink::Message>(&mut self, m: &M) {
        let port = &mut self.port;

        if let Err(_) =
            mavlink::write_versioned_msg(port, mavlink::MavlinkVersion::V2, self.mav_header, m)
        {
            fmt::trace!("mavlink write error")
        }
        self.mav_header.sequence = self.mav_header.sequence.overflowing_add(1).0;
    }

    fn send_heatbeat(&mut self) {
        use mavlink::common::*;
        let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_GENERIC,
            autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_STANDBY,
            mavlink_version: 0x3,
        });
        self.send_message(&msg)
    }

    fn send_attitude(&mut self, tnow_ms: u64, ins: &INS) {
        use mavlink::common::*;
        let (roll, pitch, yaw) = ins.get_rpy();
        let msg = MavMessage::ATTITUDE(ATTITUDE_DATA {
            roll,
            pitch,
            yaw,
            time_boot_ms: tnow_ms as u32,
            rollspeed: 0.0,
            pitchspeed: 0.0,
            yawspeed: 0.0,
        });
        self.send_message(&msg)
    }

    fn update_send(&mut self, ins: &INS) {
        let tnow = embassy_time::Instant::now();

        for i in 0..self.message_rates.len() {
            let dt = (tnow - self.last_update_time[i]).as_micros() as f32;
            let rate_hz = self.message_rates[i].1;
            if rate_hz == 0 {
                continue;
            }
            if dt < (1_000_000 / rate_hz) as f32 {
                continue;
            }
            match self.message_rates[i].0 {
                MSGID::HEARTBEAT => self.send_heatbeat(),
                MSGID::ATTITUDE => self.send_attitude(tnow.as_millis(), ins),
                _ => (),
            }
            self.last_update_time[i] = tnow;
        }
    }

    fn send_ack(&mut self, command: mavlink::common::MavCmd, result: mavlink::common::MavResult) {
        use mavlink::common::*;
        let msg = MavMessage::COMMAND_ACK(COMMAND_ACK_DATA { command, result });

        self.send_message(&msg);
    }

    fn handle_command_int(
        &mut self,
        msg: &mavlink::common::COMMAND_INT_DATA,
    ) -> mavlink::common::MavResult {
        match msg.command {
            _ => {
                fmt::display_debug!("recieved command int {:?}", msg.command);
            }
        }

        return mavlink::common::MavResult::MAV_RESULT_UNSUPPORTED;
    }

    fn set_message_interval(
        &mut self,
        message_id: u32,
        interval: u32,
        _response_target: u8,
    ) -> mavlink::common::MavResult {
        let interval_hz = 1_000_000/interval;
        if interval_hz > 50 {
            return mavlink::common::MavResult::MAV_RESULT_UNSUPPORTED
        }
        for i in 0..self.message_rates.len() {
            let id = self.message_rates[i].0;
            if id as u32 == message_id {
                self.message_rates[i].1 = interval_hz;
                return mavlink::common::MavResult::MAV_RESULT_ACCEPTED
            }
        }
        return mavlink::common::MavResult::MAV_RESULT_FAILED;
    }

    fn handle_command_long(
        &mut self,
        msg: &mavlink::common::COMMAND_LONG_DATA,
    ) -> mavlink::common::MavResult {
        match msg.command {
            mavlink::common::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL => {
                self.set_message_interval(msg.param1 as u32, msg.param2 as u32, msg.param3 as u8)
            }
            _ => {
                fmt::display_debug!("recieved command long {:?}", msg.command);
                return mavlink::common::MavResult::MAV_RESULT_UNSUPPORTED;
            }
        }
    }

    fn read_msg(&mut self) -> Result<(), mavlink::error::MessageReadError> {
        let r: (_, mavlink::common::MavMessage) =
            mavlink::read_versioned_msg(&mut self.port, mavlink::MavlinkVersion::V2)?;
        match r.1 {
            mavlink::common::MavMessage::COMMAND_LONG(cmdlong) => {
                let r = self.handle_command_long(&cmdlong);
                self.send_ack(cmdlong.command, r);
            }
            mavlink::common::MavMessage::COMMAND_INT(cmdint) => {
                let r = self.handle_command_int(&cmdint);
                self.send_ack(cmdint.command, r);
            }
            _ => (),
        }
        Ok(())
    }

    fn update_recieve(&mut self) {
        if let Err(_e) = self.read_msg() {
            fmt::trace!("mavlink: read error");
        }
    }
}

fn serial_comm(_cts: &GlobalContext) {
    let gcs = &mut _cts.gcs.borrow_mut();

    gcs.update_recieve();
    gcs.update_send(&_cts.ins.borrow());
}

fn get_angles(dcm: &nalgebra::Matrix3<f32>) -> (f32, f32, f32) {
    use micromath::F32Ext;
    let c0 = F32Ext::sqrt(F32Ext::powf(dcm.m31, 2.0) + F32Ext::powf(dcm.m33, 2.0));
    let pitch = F32Ext::atan(dcm.m32 / c0);
    let roll = -F32Ext::atan2(dcm.m31, dcm.m33);
    let yaw = F32Ext::atan2(dcm.m12, dcm.m22);

    (roll, pitch, yaw)
}

const TO_DEG: f32 = 180.0 / 3.1415926;

fn to_deg(mut a_rad: (f32, f32, f32)) -> (f32, f32, f32) {
    a_rad.0 *= TO_DEG;
    a_rad.1 *= TO_DEG;
    a_rad.2 *= TO_DEG;
    a_rad
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
    fmt::unwrap!(_spawner.spawn(hal::boards::serial0_runner(hal::serial::Config::default())));

    let ringbuf = static_cell::make_static!(ringbuf::StaticRb::<
        (nalgebra::Vector3<f32>, nalgebra::Vector3<f32>),
        5,
    >::default());
    let (prod, con) = ringbuf.split_ref();
    fmt::unwrap!(_spawner.spawn(hal::boards::imu_task(prod)));

    let context = GlobalContext {
        gcs: GcsMavlink::new().into(),
        ins: INS::new(con).into(),
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
