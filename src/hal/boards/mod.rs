pub struct Board;
use super::*;

#[cfg(feature = "f407vg")]
mod f407vg;

#[cfg(feature = "f103c8")]
mod f103c8;

#[cfg(feature = "std")]
mod std;

pub type GenericSerial = impl serial::GenericSerialTrait;

pub trait GenericBoard {
    fn init();
}
