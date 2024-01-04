pub struct Board;
use super::*;
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "f407vg")]
mod f407vg;

#[cfg(feature = "f103c8")]
mod f103c8;

#[cfg(feature = "std")]
mod std;

pub type GenericSerial = impl serial::GenericSerialTrait;
pub type GpioOutput = impl OutputPin;

pub trait GenericBoard {
    fn init();
}
