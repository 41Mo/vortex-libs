#![allow(dead_code)]
use crate::fmt;
use mavlink;
use mavlink::common as mavproto;
use serial_manager as serial;

pub struct GcsMavlink {
    port: serial::SerialPort,
    mav_header: mavlink::MavHeader,
    last_update_time: [embassy_time::Instant; 2],
    message_rates: [(MSGID, u32); 2],
}

/// Mavlink messages that we are sending at sertaint rate
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum MSGID {
    HEARTBEAT = 1,
}

impl GcsMavlink {
    pub fn new() -> Self {
        let port = serial::find_by_protocol(serial::Protocol::MavlinkV2).unwrap();
        Self {
            port,
            mav_header: mavlink::MavHeader {
                system_id: 1,
                component_id: mavproto::MavComponent::MAV_COMP_ID_USER1,
                sequence: 0,
            },
            last_update_time: [embassy_time::Instant::from_micros(0); 2],
            message_rates: [(MSGID::HEARTBEAT, 1)],
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
        use mavproto::*;
        let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_GENERIC,
            autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 0x3,
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
                #[allow(unreachable_patterns)]
                _ => (),
            }
            self.last_update_time[i] = tnow;
        }
    }

    fn send_ack(&mut self, command: mavproto::MavCmd, result: mavproto::MavResult) {
        use mavproto::*;
        let msg = MavMessage::COMMAND_ACK(COMMAND_ACK_DATA { command, result });

        self.send_message(&msg);
    }

    fn handle_command_int(&mut self, msg: &mavproto::COMMAND_INT_DATA) -> mavproto::MavResult {
        match msg.command {
            _ => {
                fmt::display_debug!("recieved command int {:?}", msg.command);
            }
        }

        return mavproto::MavResult::MAV_RESULT_UNSUPPORTED;
    }

    fn set_message_interval(
        &mut self,
        message_id: u32,
        interval: u32,
        _response_target: u8,
    ) -> mavproto::MavResult {
        let interval_hz = 1_000_000 / interval;
        if interval_hz > 50 {
            return mavproto::MavResult::MAV_RESULT_UNSUPPORTED;
        }
        for i in 0..self.message_rates.len() {
            let id = self.message_rates[i].0;
            if id as u32 == message_id {
                self.message_rates[i].1 = interval_hz;
                return mavproto::MavResult::MAV_RESULT_ACCEPTED;
            }
        }
        return mavproto::MavResult::MAV_RESULT_FAILED;
    }

    fn handle_command_long(&mut self, msg: &mavproto::COMMAND_LONG_DATA) -> mavproto::MavResult {
        match msg.command {
            mavproto::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL => {
                self.set_message_interval(msg.param1 as u32, msg.param2 as u32, msg.param3 as u8)
            }
            _ => {
                fmt::display_debug!("recieved command long {:?}", msg.command);
                return mavproto::MavResult::MAV_RESULT_UNSUPPORTED;
            }
        }
    }

    fn read_msg(&mut self) -> Result<(), mavlink::error::MessageReadError> {
        let r: (_, mavproto::MavMessage) =
            mavlink::read_versioned_msg(&mut self.port, mavlink::MavlinkVersion::V2)?;
        match r.1 {
            mavproto::MavMessage::COMMAND_LONG(cmdlong) => {
                let r = self.handle_command_long(&cmdlong);
                self.send_ack(cmdlong.command, r);
            }
            mavproto::MavMessage::COMMAND_INT(cmdint) => {
                let r = self.handle_command_int(&cmdint);
                self.send_ack(cmdint.command, r);
            }
            _ => (),
        }
        Ok(())
    }

    pub fn update_recieve(&mut self) {
        if let Err(_e) = self.read_msg() {
            // fmt::trace!("mavlink: read error");
        }
    }
}

impl Into<u32> for MSGID {
    fn into(self) -> u32 {
        self as u32
    }
}
