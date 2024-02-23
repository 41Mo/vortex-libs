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
pub type RingBufWriteRef = embassy_sync::pipe::Writer<'static, CriticalSectionRawMutex, RINGBUF_SIZE>;
pub type RingBufReadRef = embassy_sync::pipe::Reader<'static, CriticalSectionRawMutex, RINGBUF_SIZE>;
pub type SerialRingBuf = embassy_sync::pipe::Pipe<CriticalSectionRawMutex, RINGBUF_SIZE>;

pub struct SerialPort {
    reader: RingBufReadRef,
    writer: RingBufWriteRef,
}

impl embedded_hal_02::serial::Read<u8> for SerialPort {
    type Error = Error;
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let mut byte = [0u8];
        let r = self.reader.try_read(&mut byte);
        if let Err(e) = r {
            return Err(nb::Error::Other(Error::NoData));
        }
        Ok(byte[0])
    }
}

impl embedded_hal_02::serial::Write<u8> for SerialPort {
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        let r = self.writer.try_write(&[word]);
        if let Err(e) = r {
            return Err(nb::Error::Other(Error::BufferFull))
        }
        Ok(())
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        todo!()
    }
}

impl SerialPort {
    pub fn read_slice(&self, bytes: &mut [u8]) -> Option<usize> {
        let r = self.reader.try_read(bytes);
        if let Err(e) = r {
            return None;
        }
        Some(r.unwrap())
    }
    pub fn write_slice(&self, bytes: &[u8]) -> Option<usize> {
        let r = self.writer.try_write(bytes);
        if let Err(e) = r {
            return None
        }
        Some(r.unwrap())
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
                static_cell::make_static!(SerialRingBuf::new());

            let sc2: &'static mut SerialRingBuf =
                static_cell::make_static!(SerialRingBuf::new());
            (sc1, sc2)
        }
        1 => {
            let sc1: &'static mut SerialRingBuf =
                static_cell::make_static!(SerialRingBuf::new());

            let sc2: &'static mut SerialRingBuf =
                static_cell::make_static!(SerialRingBuf::new());
            (sc1, sc2)
        }
        2 => {
            let sc1: &'static mut SerialRingBuf =
                static_cell::make_static!(SerialRingBuf::new());

            let sc2: &'static mut SerialRingBuf =
                static_cell::make_static!(SerialRingBuf::new());
            (sc1, sc2)
        }
        3 => {
            let sc1: &'static mut SerialRingBuf =
                static_cell::make_static!(SerialRingBuf::new());

            let sc2: &'static mut SerialRingBuf =
                static_cell::make_static!(SerialRingBuf::new());
            (sc1, sc2)
        }
        4 => {
            let sc1: &'static mut SerialRingBuf =
                static_cell::make_static!(SerialRingBuf::new());

            let sc2: &'static mut SerialRingBuf =
                static_cell::make_static!(SerialRingBuf::new());
            (sc1, sc2)
        }
        5 => {
            let sc1: &'static mut SerialRingBuf =
                static_cell::make_static!(SerialRingBuf::new());

            let sc2: &'static mut SerialRingBuf =
                static_cell::make_static!(SerialRingBuf::new());
            (sc1, sc2)
        }
        6 => {
            let sc1: &'static mut SerialRingBuf =
                static_cell::make_static!(SerialRingBuf::new());

            let sc2: &'static mut SerialRingBuf =
                static_cell::make_static!(SerialRingBuf::new());
            (sc1, sc2)
        }
        7 => {
            let sc1: &'static mut SerialRingBuf =
                static_cell::make_static!(SerialRingBuf::new());

            let sc2: &'static mut SerialRingBuf =
                static_cell::make_static!(SerialRingBuf::new());
            (sc1, sc2)
        }
        _ => panic!("SERIAL portnum out of bounds"),
    };
    let (r_rb_r, r_rb_w) = rb1.split();
    let (w_rb_r, w_rb_w) = rb2.split();
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
