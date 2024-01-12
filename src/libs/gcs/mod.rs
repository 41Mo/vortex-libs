#![allow(dead_code)]
use mavlink;
use crate::libs::ins::INS;
use crate::hal::serial;
use crate::fmt;

pub struct GcsMavlink {
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

impl GcsMavlink
{
    pub fn new() -> Self {
        let port = serial::find_by_protocol(serial::Protocol::MavlinkV2).unwrap();
        Self {
            port,
            mav_header: mavlink::MavHeader {
                system_id: 1,
                component_id: 1,
                sequence: 0,
            },
            last_update_time: [embassy_time::Instant::from_micros(0); 2],
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

    pub fn update_send(&mut self, ins: &INS) {
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
                #[allow(unreachable_patterns)]
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
        let interval_hz = 1_000_000 / interval;
        if interval_hz > 50 {
            return mavlink::common::MavResult::MAV_RESULT_UNSUPPORTED;
        }
        for i in 0..self.message_rates.len() {
            let id = self.message_rates[i].0;
            if id as u32 == message_id {
                self.message_rates[i].1 = interval_hz;
                return mavlink::common::MavResult::MAV_RESULT_ACCEPTED;
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

    pub fn update_recieve(&mut self) {
        if let Err(_e) = self.read_msg() {
            fmt::trace!("mavlink: read error");
        }
    }
}

