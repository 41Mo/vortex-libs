#![allow(unused)]
use crate::hal::serial;
use crate::hal::*;
use crate::libs::fmt;
use async_io::Async;
use serial::*;
use std::net::UdpSocket;
use std::sync::Arc;

impl super::GenericBoard for super::Board {
    fn init() {}
}

pub mod hw_tasks {
    use super::*;
    #[embassy_executor::task]
    pub async fn serial1_runner(cfg: serial::Config) {}
}

