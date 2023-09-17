pub struct Board;

#[cfg(feature = "f407vg")]
mod f407vg;

#[cfg(feature = "std")]
mod std;

pub trait GenericSerialTrait {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()>;
    async fn write(&mut self, buf: &[u8]) -> Result<usize, ()>;
    fn split(self) -> (impl embedded_io_async::Write, impl embedded_io_async::Read);
}

pub type GenericSerial = impl GenericSerialTrait;

pub trait GenericBoard {
    fn init();
}
