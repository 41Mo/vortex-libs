#![allow(unused_variables)]
#![allow(dead_code)]
use crate::{hal::boards::RunnerFType};
use core::{cell::RefCell, future::Future};
use embassy_executor::SpawnToken;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use heapless::Vec;
use ringbuf::Rb;

const MAX_SERIALNUM: usize = 8;

static SERIAL_MANAGER: SerialManager = SerialManager {
    ports: Mutex::new(RefCell::new(Vec::new())),
};

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum Protocol {
    Test,
}

struct SerialManager {
    ports: Mutex<CriticalSectionRawMutex, RefCell<Vec<SerialWrapper, MAX_SERIALNUM>>>,
}
pub type SerialPort = ringbuf::StaticConsumer<'static, u8, 512>;
type SerialPortRbW = ringbuf::StaticProducer<'static, u8, 512>;
type SerialRingBuf = ringbuf::StaticRb<u8, 512>;

pub struct SerialWrapper {
    protocol: Protocol,
    rb_read_ref: Option<SerialPort>,
    rb_write_ref: Option<SerialPortRbW>,
    runner: Option<RunnerFType>,
}

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub baud: u32,
    pub swap_rx_tx: bool,
}

impl Config {
    pub fn default() -> Self {
        Self {
            baud: 57_600,
            swap_rx_tx: false,
        }
    }

    pub fn baud(&mut self, b: u32) -> Self {
        self.baud = b;
        *self
    }

    pub fn swap_rx_tx(&mut self, do_swap: bool) -> Self {
        todo!()
    }
}

pub fn find_port_runner(protocol: Protocol) -> Option<(RunnerFType, ringbuf::Producer<u8, &'static SerialRingBuf>)> {
    let ports = &SERIAL_MANAGER
        .ports
        .try_lock()
        .expect("SerialManager: lock should never fail");
    let ports = &mut ports.borrow_mut();

    let protocol = &protocol;
    match ports.iter().position(|e| e.protocol == *protocol) {
        Some(n) => {
            let runner = ports[n].runner.take().unwrap();
            let rb = ports[n].rb_write_ref.take().unwrap();
            Some((runner, rb))
        },
        None => None,
    }
}

pub fn find_by_protocol(protocol: Protocol) -> Option<SerialPort> {
    let ports = &SERIAL_MANAGER
        .ports
        .try_lock()
        .expect("SerialManager: lock should never fail");
    let mut ports = ports.borrow_mut();

    let protocol = &protocol;
    match ports.iter().position(|e| e.protocol == *protocol) {
        Some(n) => ports[n].rb_read_ref.take(),
        None => None,
    }
}

pub(in crate::hal) fn bind_port(
    protocol: Protocol,
    runner: RunnerFType,
    rb_r: SerialPort,
    rb_w: SerialPortRbW,
) {
    let ports = &SERIAL_MANAGER
        .ports
        .try_lock()
        .expect("SerialManager: lock should never fail");
    let mut ports = ports.borrow_mut();
    if ports
        .push(SerialWrapper {
            protocol,
            runner: Some(runner),
            rb_read_ref: Some(rb_r),
            rb_write_ref: Some(rb_w)
        })
        .is_err()
    {
        panic!("unable to bind port")
    }
}

#[cfg(feature = "stm32")]
impl Into<embassy_stm32::usart::Config> for Config {
    fn into(self) -> embassy_stm32::usart::Config {
        let mut cfg = embassy_stm32::usart::Config::default();
        cfg.baudrate = self.baud;
        cfg
    }
}

#[cfg(test)]
mod tests {
    #![allow(unused_imports)]

    use super::*;
    use static_cell::make_static;
    use std::ptr::addr_of;

    #[test]
    fn feels_good() {
        /*

        mod boards/some_board {
            // inside board support module
             fn init_serial() -> GenericSerial {
                  ....
             }

            // bining port init function and port protocol, so later we can find this port by
            // protocol
            let port_init = |_: Config| {init_serial()};
            bind_port(Protocol::Test, port_init)
        }

        mod main {
             // find wrapper structure
             let port = find_by_protocol(Protocol::Test);

             // real initialization of peripherals goes here
             let uart = port.begin(Config::default());

             loop {
                 uart.read(..).await

                 uart.write(..).await
             }
        }


        */
    }

    #[test]
    fn config_nice() {
        let cfg = Config::default().baud(115_200);
        println!("cfg {:?}", cfg);
    }
}
