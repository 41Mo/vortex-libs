pub struct Board;
use super::{*, serial::serial_manager::Config};
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "f407vg")]
mod f407vg;

#[cfg(feature = "f103c8")]
pub mod f103c8;

#[cfg(feature = "std")]
mod std;

pub type GpioOutput = impl OutputPin;
pub type RunnerRType = embassy_executor::SpawnToken<impl Sized>;
pub type RunnerFType = impl FnOnce(Config, ringbuf::StaticProducer<'static, u8, 512>) -> RunnerRType;

pub trait GenericBoard {
    fn init();
}
