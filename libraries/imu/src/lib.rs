#![no_std]
#![feature(type_alias_impl_trait)]
use nalgebra;
use ringbuf;

const BUF_MAX: usize = 5;
type RBData = (nalgebra::Vector3<f32>, nalgebra::Vector3<f32>, u16);
pub type Consumer = ringbuf::StaticConsumer<'static, RBData, BUF_MAX>;
pub type Producer = ringbuf::StaticProducer<'static, RBData, BUF_MAX>;

#[repr(u8)]
pub enum ImuInstance {
    IMU1,
    IMU2,
    IMU3,
}

struct ImuBuf {
    consumer: Option<Consumer>,
    producer: Option<Producer>,
}

impl ImuBuf {
    fn new() -> Self {
        let a: &'static mut ringbuf::StaticRb<RBData, BUF_MAX> =
            static_cell::make_static!(ringbuf::StaticRb::<RBData, BUF_MAX>::default());
        let (producer, consumer) = a.split_ref();
        Self {
            producer: Some(producer),
            consumer: Some(consumer),
        }
    }
}

struct IMU1;
struct IMU2;
struct IMU3;

use core::cell::RefCell;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
trait ImuBufSingleton: Sync {
    fn get_singleton() -> &'static Mutex<CriticalSectionRawMutex, RefCell<Option<ImuBuf>>> {
        static IMU_BUF: Mutex<CriticalSectionRawMutex, RefCell<Option<ImuBuf>>> =
            Mutex::new(RefCell::new(None));
        IMU_BUF.lock(|v| {
            let mut rc = v.borrow_mut();
            if rc.is_none() {
                rc.replace(ImuBuf::new());
            }
        });
        &IMU_BUF
    }
}

impl ImuBufSingleton for IMU1 {}
impl ImuBufSingleton for IMU2 {}
impl ImuBufSingleton for IMU3 {}

pub fn get_imu_consumer(instance: ImuInstance) -> Option<Consumer> {
    match instance {
        ImuInstance::IMU1 => {
            IMU1::get_singleton().lock(|v| v.borrow_mut().as_mut().unwrap().consumer.take())
        }
        ImuInstance::IMU2 => {
            IMU2::get_singleton().lock(|v| v.borrow_mut().as_mut().unwrap().consumer.take())
        },
        ImuInstance::IMU3 => {
            IMU3::get_singleton().lock(|v| v.borrow_mut().as_mut().unwrap().consumer.take())
        },
    }
}

pub fn get_imu_producer(instance: ImuInstance) -> Option<Producer> {
    match instance {
        ImuInstance::IMU1 => {
            IMU1::get_singleton().lock(|v| v.borrow_mut().as_mut().unwrap().producer.take())
        }
        ImuInstance::IMU2 => {
            IMU2::get_singleton().lock(|v| v.borrow_mut().as_mut().unwrap().producer.take())
        },
        ImuInstance::IMU3 => {
            IMU3::get_singleton().lock(|v| v.borrow_mut().as_mut().unwrap().producer.take())
        },
    }

}
