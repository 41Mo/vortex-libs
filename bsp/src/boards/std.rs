#![allow(unused)]
use crate::fmt;
use async_io::Async;
use serial::*;
use serial_manager as serial;
use std::net::UdpSocket;
use std::sync::Arc;

impl super::GenericBoard for super::Board {
    fn init() {}
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
    pub async fn serial_runner(portnum: u8, cfg: serial::Config) {
        let addr = std::net::SocketAddr::from_str(cfg.dev.as_str()).unwrap();
        let sock = Arc::new(Async::<UdpSocket>::bind(addr).unwrap());
        let rx = sock.clone();
        let tx = sock.clone();
        let client_addr = embassy_sync::mutex::Mutex::<
            embassy_sync::blocking_mutex::raw::NoopRawMutex,
            Option<std::net::SocketAddr>,
        >::new(None);

        let (con, prod) = fmt::unwrap!(serial_manager::find_port_rb_ref(portnum));
        let name = stringify!("Serial" + portnum);

        let reader = port_reader(rx, prod, name, &client_addr);
        let writer = port_writer(tx, con, name, &client_addr);

        embassy_futures::join::join(reader, writer).await;
    }
}
