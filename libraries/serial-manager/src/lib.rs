#![no_std]
#![feature(type_alias_impl_trait)]

use bitfield_struct::bitfield;
use core::cell::RefCell;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use heapless::Vec;

const MAX_SERIALNUM: usize = 8;

static SERIAL_MANAGER: SerialManager = SerialManager {
    ports: Mutex::new(RefCell::new(Vec::new())),
};

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
    pub fn read_slice(&mut self, bytes: &mut [u8]) -> usize {
        self.reader.pop_slice(bytes)
    }
    pub fn write(&mut self, byte: u8) -> Result<(), ()> {
        self.writer.push(byte).map_err(|_| ())
    }
    pub fn write_slice(&mut self, bytes: &[u8]) -> Result<(), ()> {
        if self.writer.push_slice(bytes) < bytes.len() {
            return Err(());
        }
        Ok(())
    }
}

pub enum Error {
    NoData,
    BufferFull,
}

struct SerialWrapper {
    protocol: u8,
    port_num: u8,
    u_rb: Option<SerialPort>,
    p_rb: Option<(RingBufReadRef, RingBufWriteRef)>,
}
#[bitfield(u8)]
pub struct Options {
    pub enable_rx: bool,
    pub enable_tx: bool,
    pub swap_rx_tx: bool,
    #[bits(5)]
    __: u8,
}

#[cfg(not(feature = "std"))]
mod config {
    use crate::Options;
    #[derive(Clone, Copy, Debug)]
    pub struct Config {
        pub baud: u32,
        pub options: Options,
    }

    impl Config {
        pub fn default() -> Self {
            Self {
                baud: 57_600,
                options: Options::new()
                    .with_enable_rx(true)
                    .with_enable_tx(true)
                    .with_swap_rx_tx(false),
            }
        }
    }
}

#[cfg(feature = "std")]
mod config {
    use crate::Options;
    #[derive(Clone, Debug)]
    pub struct Config {
        pub dev: heapless::String<255>,
        pub options: Options,
    }
    impl Config {
        pub fn default() -> Self {
            Self {
                dev: heapless::String::new(),
                options: Options::new()
                    .with_enable_rx(true)
                    .with_enable_tx(true)
                    .with_swap_rx_tx(false),
            }
        }
    }
}

// reexport
pub use config::*;

pub fn find_by_protocol(protocol: u8) -> Option<SerialPort> {
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

pub fn bind_ports(port_to_proto_binding: &[(u8, u8)]) {
    if port_to_proto_binding.len() > MAX_SERIALNUM {
        panic!("binding len > MAX_SERIALNUM");
    }

    for i in port_to_proto_binding {
        bind_port(i.0, i.1);
    }
}

pub fn bind_port(port_num: u8, protocol: u8) {
    let ports = &SERIAL_MANAGER
        .ports
        .try_lock()
        .expect("SerialManager: lock should never fail");
    let mut ports = ports.borrow_mut();
    let (rb1, rb2) = match port_num {
        0 => {
            let sc1: &'static mut SerialRingBuf =
                static_cell::make_static!(ringbuf::StaticRb::default());

            let sc2: &'static mut SerialRingBuf =
                static_cell::make_static!(ringbuf::StaticRb::default());
            (sc1, sc2)
        }
        1 => {
            let sc1: &'static mut SerialRingBuf =
                static_cell::make_static!(ringbuf::StaticRb::default());

            let sc2: &'static mut SerialRingBuf =
                static_cell::make_static!(ringbuf::StaticRb::default());
            (sc1, sc2)
        }
        2 => {
            let sc1: &'static mut SerialRingBuf =
                static_cell::make_static!(ringbuf::StaticRb::default());

            let sc2: &'static mut SerialRingBuf =
                static_cell::make_static!(ringbuf::StaticRb::default());
            (sc1, sc2)
        }
        3 => {
            let sc1: &'static mut SerialRingBuf =
                static_cell::make_static!(ringbuf::StaticRb::default());

            let sc2: &'static mut SerialRingBuf =
                static_cell::make_static!(ringbuf::StaticRb::default());
            (sc1, sc2)
        }
        4 => {
            let sc1: &'static mut SerialRingBuf =
                static_cell::make_static!(ringbuf::StaticRb::default());

            let sc2: &'static mut SerialRingBuf =
                static_cell::make_static!(ringbuf::StaticRb::default());
            (sc1, sc2)
        }
        5 => {
            let sc1: &'static mut SerialRingBuf =
                static_cell::make_static!(ringbuf::StaticRb::default());

            let sc2: &'static mut SerialRingBuf =
                static_cell::make_static!(ringbuf::StaticRb::default());
            (sc1, sc2)
        }
        6 => {
            let sc1: &'static mut SerialRingBuf =
                static_cell::make_static!(ringbuf::StaticRb::default());

            let sc2: &'static mut SerialRingBuf =
                static_cell::make_static!(ringbuf::StaticRb::default());
            (sc1, sc2)
        }
        7 => {
            let sc1: &'static mut SerialRingBuf =
                static_cell::make_static!(ringbuf::StaticRb::default());

            let sc2: &'static mut SerialRingBuf =
                static_cell::make_static!(ringbuf::StaticRb::default());
            (sc1, sc2)
        }
        _ => panic!("SERIAL portnum out of bounds"),
    };
    let (r_rb_w, r_rb_r) = rb1.split_ref();
    let (w_rb_w, w_rb_r) = rb2.split_ref();
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
        let cfg = crate::config::default().baud(115_200);
        println!("cfg {:?}", cfg);
    }
}
