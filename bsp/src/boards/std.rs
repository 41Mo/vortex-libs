#![allow(unused)]
use crate::fmt;
use async_io::Async;
use serial::*;
use serial_manager as serial;
use std::net::UdpSocket;
use std::sync::Arc;

impl super::GenericBoard for super::Board {
    fn init() {
        serial0_bind();
        serial1_bind();
        serial2_bind();
    }
}

fn serial0_bind() {
    fmt::trace!("binding serial0");
    let sc1: &'static mut serial_manager::SerialRingBuf =
        static_cell::make_static!(ringbuf::StaticRb::default());

    let sc2: &'static mut serial_manager::SerialRingBuf =
        static_cell::make_static!(ringbuf::StaticRb::default());
    let (p1, c1) = sc1.split_ref();
    let (p2, c2) = sc2.split_ref();
    serial_manager::bind_port(serial_manager::Protocol::MavlinkV2, 0, c1, p1, c2, p2);
}
fn serial1_bind() {
    fmt::trace!("binding serial1");
    let sc1: &'static mut serial_manager::SerialRingBuf =
        static_cell::make_static!(ringbuf::StaticRb::default());

    let sc2: &'static mut serial_manager::SerialRingBuf =
        static_cell::make_static!(ringbuf::StaticRb::default());
    let (p1, c1) = sc1.split_ref();
    let (p2, c2) = sc2.split_ref();
    serial_manager::bind_port(serial_manager::Protocol::MavlinkV2, 1, c1, p1, c2, p2);
}
fn serial2_bind() {
    fmt::trace!("binding serial2");
    let sc1: &'static mut serial_manager::SerialRingBuf =
        static_cell::make_static!(ringbuf::StaticRb::default());

    let sc2: &'static mut serial_manager::SerialRingBuf =
        static_cell::make_static!(ringbuf::StaticRb::default());
    let (p1, c1) = sc1.split_ref();
    let (p2, c2) = sc2.split_ref();
    serial_manager::bind_port(serial_manager::Protocol::MavlinkV2, 2, c1, p1, c2, p2);
}

pub mod hw_tasks {
    use core::str::FromStr;

    async fn port_reader(
        sock: Arc<Async<UdpSocket>>,
        mut producer: serial_manager::RingBufWriteRef,
        name: &str,
        client_addr: &embassy_sync::mutex::Mutex<
            embassy_sync::blocking_mutex::raw::NoopRawMutex,
            Option<std::net::SocketAddr>,
        >,
    ) {
        let mut buf = [0u8; 2048];
        let mut client_connected = false;
        loop {
            let (nbytes, addr) = match sock.recv_from(&mut buf).await {
                Ok(v) => v,
                Err(e) => {
                    fmt::error!("{} read err {}", name, e);
                    continue;
                }
            };
            if !client_connected {
                let mut a = client_addr.lock().await;
                a.replace(addr);
                fmt::debug!("client connected {}", addr);
                client_connected = true;
            }
            producer.push_slice(&buf[..nbytes]);
        }
    }

    async fn port_writer(
        sock: Arc<Async<UdpSocket>>,
        mut consumer: serial_manager::RingBufReadRef,
        name: &str,
        client_addr: &embassy_sync::mutex::Mutex<
            embassy_sync::blocking_mutex::raw::NoopRawMutex,
            Option<std::net::SocketAddr>,
        >,
    ) -> ! {
        let client_addr = loop {
            embassy_time::Timer::after_secs(1).await;
            let a = client_addr.lock().await;
            if a.is_some() {
                break a.as_ref().unwrap().clone();
            }
            drop(a);
        };
        let mut buf = [0u8; 2048];
        loop {
            let nbytes = consumer.pop_slice(&mut buf);
            if nbytes == 0 {
                embassy_time::Timer::after_millis(10).await;
                continue;
            }
            match sock.send_to(&buf, client_addr).await {
                Ok(r) => (),
                Err(e) => {
                    fmt::error!("{} write err {}", name, e);
                    continue;
                }
            }
        }
    }

    use super::*;
    #[embassy_executor::task]
    pub async fn serial0_runner(cfg: serial::Config) {
        let addr = std::net::SocketAddr::from_str(cfg.dev.as_str()).unwrap();
        let sock = Arc::new(Async::<UdpSocket>::bind(addr).unwrap());
        let rx = sock.clone();
        let tx = sock.clone();
        let client_addr = embassy_sync::mutex::Mutex::<
            embassy_sync::blocking_mutex::raw::NoopRawMutex,
            Option<std::net::SocketAddr>,
        >::new(None);

        let (con, prod) = fmt::unwrap!(serial_manager::find_port_rb_ref(0));
        let name = "Serial0";

        let reader = port_reader(rx, prod, name, &client_addr);
        let writer = port_writer(tx, con, name, &client_addr);

        embassy_futures::join::join(reader, writer).await;
    }

    #[embassy_executor::task]
    pub async fn serial1_runner(cfg: serial::Config) {
        let addr = std::net::SocketAddr::from_str(cfg.dev.as_str()).unwrap();
        let sock = Arc::new(Async::<UdpSocket>::bind(addr).unwrap());
        let rx = sock.clone();
        let tx = sock.clone();
        let client_addr = embassy_sync::mutex::Mutex::<
            embassy_sync::blocking_mutex::raw::NoopRawMutex,
            Option<std::net::SocketAddr>,
        >::new(None);

        let (con, prod) = fmt::unwrap!(serial_manager::find_port_rb_ref(1));
        let name = "Serial1";

        let reader = port_reader(rx, prod, name, &client_addr);
        let writer = port_writer(tx, con, name, &client_addr);

        embassy_futures::join::join(reader, writer).await;
    }

    #[embassy_executor::task]
    pub async fn serial2_runner(cfg: serial::Config) {
        let addr = std::net::SocketAddr::from_str(cfg.dev.as_str()).unwrap();
        let sock = Arc::new(Async::<UdpSocket>::bind(addr).unwrap());
        let rx = sock.clone();
        let tx = sock.clone();
        let client_addr = embassy_sync::mutex::Mutex::<
            embassy_sync::blocking_mutex::raw::NoopRawMutex,
            Option<std::net::SocketAddr>,
        >::new(None);

        let (con, prod) = fmt::unwrap!(serial_manager::find_port_rb_ref(2));
        let name = "Serial2";

        let reader = port_reader(rx, prod, name, &client_addr);
        let writer = port_writer(tx, con, name, &client_addr);

        embassy_futures::join::join(reader, writer).await;
    }
}
