struct Dummy();

struct DummySerial();

#[derive(Debug)]
struct DummyError();

use crate::hal::*;

impl GenericSerialTrait for DummySerial {
    fn split(self) -> (impl embedded_io_async::Write, impl embedded_io_async::Read) {
        (Dummy(), Dummy())
    }

    async fn read(&mut self, _buf: &mut [u8]) -> Result<usize, ()> {
        todo!()
    }

    async fn write(&mut self, _buf: &[u8]) -> Result<usize, ()> {
        todo!()
    }
}

impl embedded_io_async::Error for DummyError {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        todo!()
    }
}

impl embedded_io_async::ErrorType for Dummy {
    type Error = DummyError;
}

impl embedded_io_async::Write for Dummy {
    async fn write(&mut self, _buf: &[u8]) -> Result<usize, Self::Error> {
        todo!()
    }
}

impl embedded_io_async::Read for Dummy {
    async fn read(&mut self, _buf: &mut [u8]) -> Result<usize, Self::Error> {
        todo!()
    }
}

fn _dummy() -> GenericSerial {
    DummySerial()
}
