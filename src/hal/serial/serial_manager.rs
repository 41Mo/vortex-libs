#![allow(unused_variables)]
#![allow(dead_code)]
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
const RINGBUF_SIZE: usize = 300;
pub(in crate::hal) type RingBufWriteRef = ringbuf::StaticProducer<'static, u8, RINGBUF_SIZE>;
pub(in crate::hal) type RingBufReadRef = ringbuf::StaticConsumer<'static, u8, RINGBUF_SIZE>;
pub(in crate::hal) type SerialRingBuf = ringbuf::StaticRb<u8, RINGBUF_SIZE>;

pub struct SerialPort {
    reader: RingBufReadRef,
    writer: RingBufWriteRef,
}

impl embedded_hal_02::serial::Read<u8> for SerialPort {
    type Error = Error;
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        if self.reader.len() == 0 {
            return Err(nb::Error::Other(Error::NoData));
        }
        Ok(self.reader.pop().unwrap())
    }
}

impl embedded_hal_02::serial::Write<u8> for SerialPort {
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.writer
            .push(word)
            .map_err(|e| nb::Error::Other(Error::BufferFull))
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        todo!()
    }
}

impl SerialPort {
    pub fn available(&self) -> usize {
        self.reader.len()
    }
    pub fn read(&mut self) -> Option<u8> {
        self.reader.pop()
    }
    pub fn write(&mut self, byte:u8) -> Result<(), ()> {
        self.writer.push(byte).map_err(|e| ())
    }
    pub fn write_slice(&mut self, bytes: &[u8]) -> Result<(), ()> {
        if self.writer.push_slice(bytes) < bytes.len() {
            return Err(())
        }
        Ok(())
    }
}

pub enum Error {
    NoData,
    BufferFull,
}

struct SerialWrapper {
    protocol: Protocol,
    port_num: u8,
    u_rb: Option<SerialPort>,
    p_rb: Option<(RingBufReadRef, RingBufWriteRef)>,
}

#[derive(Clone, Copy, Debug)]
pub struct Config {
    baud: u32,
    swap_rx_tx: bool,
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

pub fn find_by_protocol(protocol: Protocol) -> Option<SerialPort> {
    let ports = &SERIAL_MANAGER
        .ports
        .try_lock()
        .expect("SerialManager: lock should never fail");
    let mut ports = ports.borrow_mut();

    let protocol = &protocol;
    match ports.iter().position(|e| e.protocol == *protocol) {
        Some(n) => ports[n].u_rb.take(),
        None => None,
    }
}

pub(in crate::hal) fn find_port_rb_ref(port_num: u8) -> Option<(RingBufReadRef, RingBufWriteRef)> {
    let ports = &SERIAL_MANAGER
        .ports
        .try_lock()
        .expect("SerialManager: lock should never fail");
    let mut ports = ports.borrow_mut();

    match ports.iter().position(|e| e.port_num == port_num) {
        Some(n) => ports[n].p_rb.take(),
        None => None,
    }
}

pub(in crate::hal) fn bind_port(
    protocol: Protocol,
    port_num: u8,
    r_rb_r: RingBufReadRef,
    r_rb_w: RingBufWriteRef,
    w_rb_r: RingBufReadRef,
    w_rb_w: RingBufWriteRef,
) {
    let ports = &SERIAL_MANAGER
        .ports
        .try_lock()
        .expect("SerialManager: lock should never fail");
    let mut ports = ports.borrow_mut();
    if ports
        .push(SerialWrapper {
            protocol,
            port_num,
            u_rb: Some(SerialPort {
                reader: r_rb_r,
                writer: w_rb_w,
            }),
            p_rb: Some((w_rb_r, r_rb_w)),
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
