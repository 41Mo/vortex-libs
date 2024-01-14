#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]

use core::cell::RefCell;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use heapless::Vec;

const MAX_SERIALNUM: usize = 8;

static SERIAL_MANAGER: SerialManager = SerialManager {
    ports: Mutex::new(RefCell::new(Vec::new())),
};

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum Protocol {
    MavlinkV2,
}

struct SerialManager {
    ports: Mutex<CriticalSectionRawMutex, RefCell<Vec<SerialWrapper, MAX_SERIALNUM>>>,
}
const RINGBUF_SIZE: usize = 300;
pub type RingBufWriteRef = ringbuf::StaticProducer<'static, u8, RINGBUF_SIZE>;
pub type RingBufReadRef = ringbuf::StaticConsumer<'static, u8, RINGBUF_SIZE>;
pub type SerialRingBuf = ringbuf::StaticRb<u8, RINGBUF_SIZE>;

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
            .map_err(|_| nb::Error::Other(Error::BufferFull))
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
        self.writer.push(byte).map_err(|_| ())
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
    pub baud: u32,
    // swap_rx_tx: bool,
}

impl Config {
    pub fn default() -> Self {
        Self {
            baud: 57_600,
            // swap_rx_tx: false,
        }
    }

    pub fn baud(&mut self, b: u32) -> Self {
        self.baud = b;
        *self
    }

    // pub fn swap_rx_tx(&mut self, do_swap: bool) -> Self {
    //     todo!()
    // }
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

pub fn find_port_rb_ref(port_num: u8) -> Option<(RingBufReadRef, RingBufWriteRef)> {
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

pub fn bind_port(
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

#[cfg(test)]
mod tests {
    #[test]
    fn config_nice() {
        let cfg = Config::default().baud(115_200);
        println!("cfg {:?}", cfg);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
