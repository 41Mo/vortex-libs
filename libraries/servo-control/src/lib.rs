#![no_std]

use embassy_sync::{
    blocking_mutex::{self, raw::CriticalSectionRawMutex},
    channel,
};

const MAX_ITEMS_IN_QUEUE: usize = 24;
const MAX_SRV_CHAN: usize = 8;
pub type PerionMs = u16;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ChNum {
    Unknown,
    Ch1,
    Ch2,
    Ch3,
    Ch4,
    Ch5,
    Ch6,
    Ch7,
    Ch8,
}

#[derive(Debug)]
pub enum PwmDriverMsg {
    SetPeriod((ChNum, PerionMs)),
    OnOff((ChNum, bool)),
}

#[derive(PartialEq, Eq, Debug)]
pub enum ControlSurface {
    None,
    Aileron,
    Elevator,
    Throttle,
}

#[derive(Debug)]
pub struct ServoChannel {
    // min_period_ms: u16,
    trim_period_ms: u16,
    // max_period_ms: u16,
    // reverse: bool,
    srv_period_ms: u16,
    // rc_in: u16,
    chnum: ChNum,
    function: ControlSurface,
}

impl ServoChannel {
    pub fn new() -> Self {
        Self {
            chnum: ChNum::Unknown,
            function: ControlSurface::None,
            trim_period_ms:0,
            srv_period_ms: 0,
        }
    }

    pub fn set_function(mut self, f: ControlSurface) -> Self {
        self.function = f;
        self
    }

    pub fn set_trim_period(mut self, p: u16) -> Self {
        self.trim_period_ms = p;
        self
    }

    pub fn set_servo_channel(mut self, n: ChNum) -> Self {
        self.chnum = n;
        self
    }
}

static CHANNEL: channel::Channel<
    blocking_mutex::raw::CriticalSectionRawMutex,
    PwmDriverMsg,
    MAX_ITEMS_IN_QUEUE,
> = channel::Channel::new();

pub fn get_receiver(
) -> channel::Receiver<'static, CriticalSectionRawMutex, PwmDriverMsg, MAX_ITEMS_IN_QUEUE> {
    CHANNEL.receiver()
}

pub fn get_sender(
) -> channel::Sender<'static, CriticalSectionRawMutex, PwmDriverMsg, MAX_ITEMS_IN_QUEUE> {
    CHANNEL.sender()
}

// TODO: move this mod to singlethreaded vehicle implementation
pub mod single_threaded_only {
    static mut SERVO_TABLE: heapless::Vec<ServoChannel, MAX_SRV_CHAN> = heapless::Vec::new();
    use super::*;
    pub fn setup_channel(srv_ch: ServoChannel) {
        unsafe { if let Err(e) = SERVO_TABLE.push(srv_ch) {
            panic!("{:?}", e)

        } }
    }

    pub fn set_surface(cs: ControlSurface, period: u16) {
        unsafe {
            let _: () = SERVO_TABLE
                .iter_mut()
                .filter(|v| v.function == cs)
                .map(|v| {
                    v.srv_period_ms = period;
                    if let Err(e) = CHANNEL.try_send(PwmDriverMsg::SetPeriod((v.chnum, period))) {
                        panic!("{:?}", e);
                    }
                })
                .collect();
        };
    }

    pub fn get_servo_period(chnum: ChNum) -> u16 {
        unsafe {
            for e in SERVO_TABLE.iter_mut() {
                if e.chnum == chnum {
                    return e.srv_period_ms;
                }
            }
        }
        return 0;
    }

    pub fn control_surface_onoff(cs: ControlSurface, on_off: bool) {
        unsafe {
            let _: () = SERVO_TABLE
                .iter_mut()
                .filter(|v| v.function == cs)
                .map(|v| {
                    if let Err(e) = CHANNEL.try_send(PwmDriverMsg::OnOff((v.chnum, on_off))) {
                        panic!("{:?}", e)
                    }
                    v.srv_period_ms = v.trim_period_ms;
                    if let Err(e) = CHANNEL.try_send(PwmDriverMsg::SetPeriod((v.chnum, v.trim_period_ms))) {
                        panic!("{:?}", e)
                    }
                })
                .collect();
        };
    }
}

pub use single_threaded_only::*;
