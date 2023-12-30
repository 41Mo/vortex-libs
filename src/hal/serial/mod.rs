pub mod serial_manager;

pub trait GenericSerialTrait {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()>;
    async fn write(&mut self, buf: &[u8]) -> Result<usize, ()>;
    fn split(self) -> (impl embedded_io_async::Write, impl embedded_io_async::Read);
}
