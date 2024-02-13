#![no_std]
#![feature(type_alias_impl_trait)]
use ringbuf;

#[repr(C, packed)]
pub struct ImuRaw {
    pub sample_time: u64,
    pub acc: [f32; 3],
    pub gyr: [f32; 3],
}

#[repr(C, packed)]
pub struct INS {
    pub sample_time: u64,
    pub acc: [f32; 3],
    pub gyr: [f32; 3],
    pub att: [f32; 3],
}

pub enum LogMessages {
    LogImuRaw(ImuRaw),
    LogINS(INS),
}

const LOG_VERSION: u8 = 0x01;
const LOG_START: u64 = 0x676f4c6f6d313400;

#[repr(C, packed)]
struct LogHeader {
    _type: u16,
}

fn log_hdr_as_bytes<'a>(m: &'a LogHeader) -> &'a [u8] {
    unsafe {
        core::slice::from_raw_parts_mut(
            core::mem::transmute::<*const LogHeader, *mut u8>(core::ptr::addr_of!(*m)),
            core::mem::size_of::<LogHeader>(),
        )
    }
}

fn msg_as_bytes<'a, T>(m: &'a T) -> &'a [u8] {
    unsafe {
        core::slice::from_raw_parts_mut(
            core::mem::transmute::<*const T, *mut u8>(core::ptr::addr_of!(*m)),
            core::mem::size_of::<T>(),
        )
    }
}

pub fn log_msg(msg: LogMessages) {
    match msg {
        LogMessages::LogImuRaw(d) => {
            let hdr = log_hdr_as_bytes(&LogHeader { _type: 0x01 });
            let m = msg_as_bytes(&d);
            log(hdr);
            log(m);
        }
        LogMessages::LogINS(d) => {
            let hdr = log_hdr_as_bytes(&LogHeader { _type: 0x02 });
            let m = msg_as_bytes(&d);
            log(hdr);
            log(m);
        }
    };
}

pub fn get_consumer() -> Option<Consumer> {
    LoggerBuf::get_singleton().lock(|v| v.borrow_mut().as_mut().unwrap().consumer.take())
}

pub fn free() -> usize {
    LoggerBuf::get_singleton().lock(|v| {
        v.borrow_mut()
            .as_mut()
            .unwrap()
            .producer
            .as_mut()
            .unwrap()
            .free_len()
    })
}

pub fn notify_driver_state(is_ready: bool) {
    LoggerBuf::get_singleton().lock(|v| {
        let mut rc = v.borrow_mut();
        let lb = rc.as_mut().unwrap();
        lb.log_driver_ready = is_ready;
    });
}

pub fn log(data: &[u8]) {
    LoggerBuf::get_singleton().lock(|v| {
        let mut rc = v.borrow_mut();
        let lb = rc.as_mut().unwrap();
        if !lb.log_driver_ready {
            return;
        }
        let prod = lb.producer.as_mut().unwrap();
        let pushed;
        if lb.consumer.is_none() {
            pushed = prod.push_slice(data);
            lb.bytes_written = lb
                .bytes_written
                .checked_add(data.len().try_into().unwrap())
                .unwrap();
            defmt::debug!("logger: wrote {} bytes", lb.bytes_written);
        } else {
            panic!("noone took consumer, check your hardware log task");
        }

        if pushed != data.len() {
            panic!("pushed not all data")
        }
    });
}

const BUF_MAX: usize = 10*1024;
type RBData = u8;
pub type Consumer = ringbuf::StaticConsumer<'static, RBData, BUF_MAX>;
pub type Producer = ringbuf::StaticProducer<'static, RBData, BUF_MAX>;

struct LoggerBuf {
    consumer: Option<Consumer>,
    producer: Option<Producer>,
    log_driver_ready: bool,
    bytes_written: u32,
}

impl LoggerBuf {
    fn new() -> Self {
        let a: &'static mut ringbuf::StaticRb<RBData, BUF_MAX> =
            static_cell::make_static!(ringbuf::StaticRb::<RBData, BUF_MAX>::default());
        let (mut producer, consumer) = a.split_ref();
        let mut start_hdr = LOG_START.to_le_bytes();
        start_hdr[0] = LOG_VERSION;
        producer.push_slice(&start_hdr);
        Self {
            producer: Some(producer),
            consumer: Some(consumer),
            log_driver_ready: false,
            bytes_written: 0,
        }
    }
}

use core::cell::RefCell;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
trait LoggerBufSingleton: Sync {
    fn get_singleton() -> &'static Mutex<CriticalSectionRawMutex, RefCell<Option<LoggerBuf>>> {
        static LOGGER: Mutex<CriticalSectionRawMutex, RefCell<Option<LoggerBuf>>> =
            Mutex::new(RefCell::new(None));
        LOGGER.lock(|v| {
            let mut rc = v.borrow_mut();
            if rc.is_none() {
                rc.replace(LoggerBuf::new());
            }
        });
        &LOGGER
    }
}

impl LoggerBufSingleton for LoggerBuf {}
