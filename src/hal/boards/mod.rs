pub struct Board;
use super::*;

#[cfg(feature = "f407vg")]
mod f407vg;

#[cfg(feature = "std")]
mod std;

pub type GenericSerial = impl serial::GenericSerialTrait;

pub trait GenericBoard {
    fn init();
}
