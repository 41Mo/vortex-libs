pub struct Board;

#[cfg(feature="f407ve")]
mod f407ve;

#[cfg(feature="std")]
mod std;

pub trait  GenericBoard {
    fn init();
}
