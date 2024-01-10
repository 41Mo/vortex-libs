pub struct Board;

#[cfg(feature = "f407vg")]
mod f407vg;

#[cfg(feature = "f103c8")]
pub mod f103c8;
#[allow(unused)]
use super::*;

#[allow(unused)]
#[cfg(feature = "f103c8")]
pub use f103c8::hw_tasks::*;


#[cfg(feature = "std")]
mod std;

pub trait GenericBoard {
    fn init();
}
