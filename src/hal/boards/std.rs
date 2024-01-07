use crate::hal::*;
use serial::*;
use async_io::Async;
use std::net::UdpSocket;
use std::sync::Arc;
use serial::serial_manager::Protocol;

#[path = "../gpio/dummy_gpio.rs"]
mod dummy_gpio;

#[derive(Debug)]
struct Serial<'a>(Arc<Async<UdpSocket>>, &'a str);

#[derive(Debug)]
struct SerialHalve<'a>(Arc<Async<UdpSocket>>, &'a str);

impl<'a> GenericSerialTrait for Serial<'a> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
        self.0
            .recv(buf)
            .await
            .map_err(|e| crate::fmt::debug!("{} read err: {}", self.1, e))
    }
    async fn write(&mut self, buf: &[u8]) -> Result<usize, ()> {
        crate::fmt::debug!("send to {}", self.1);
        self.0
            .send(buf)
            .await
            .map_err(|e| crate::fmt::debug!("{} read err: {}", self.1, e))
    }
    fn split(
        self,
    ) -> (
        impl embedded_io_async::Write + 'a,
        impl embedded_io_async::Read + 'a,
    ) {
        let tx = SerialHalve(self.0.clone(), self.1);
        let rx = SerialHalve(self.0.clone(), self.1);

        (tx, rx)
    }
}

impl<'a> embedded_io_async::ErrorType for SerialHalve<'a> {
    type Error = std::io::Error;
}

impl<'a> embedded_io_async::Read for SerialHalve<'a> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, std::io::Error> {
        self.0.recv(buf).await
    }
}

impl<'a> embedded_io_async::Write for SerialHalve<'a> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, std::io::Error> {
        self.0.send(buf).await
    }
}

impl super::GenericBoard for super::Board {
    fn init() {
        let s1 = |_: serial_manager::Config| init_serial("127.0.0.1:25001");
        serial_manager::bind_port(Protocol::Test, s1);
    }
}

fn init_serial(addr: &'static str) -> GenericSerial {
    let sock = std::net::UdpSocket::bind("0.0.0.0:0").unwrap();
    sock.connect(addr).expect("unable to connect");
    let p: Arc<Async<UdpSocket>> = Arc::new(Async::new(sock).unwrap());
    Serial(p, addr)
}
