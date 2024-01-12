pub struct Board;

pub trait GenericBoard {
    fn init();
}

#[allow(unused)]
use super::*;


#[cfg(feature = "f103c8")]
pub mod f103c8;
#[allow(unused)]
#[cfg(feature = "f103c8")]
pub use f103c8::hw_tasks::*;

#[cfg(feature = "h743vi")]
pub mod h743vi;
#[allow(unused)]
#[cfg(feature = "h743vi")]
pub use h743vi::hw_tasks::*;


#[cfg(feature = "std")]
mod std;
#[allow(unused)]
#[cfg(feature = "std")]
pub use std::hw_tasks::*;

#[cfg(feature = "f407vg")]
mod f407vg;

