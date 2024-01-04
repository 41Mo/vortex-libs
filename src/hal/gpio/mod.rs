#![allow(unused_variables)]
#![allow(dead_code)]
use core::cell::RefCell;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use heapless::Vec;
use super::GpioOutput;

const MAX_GPIO:usize = 5;

#[derive(Clone, Copy, Debug)]
pub struct Config();

impl Config {
    pub fn default() -> Self {
        Self()
    }
}

pub fn find_by_number(number: usize, cfg: Config) -> Option<GpioOutput> {
    let gpios = &GPIO_MANAGER
        .ports
        .try_lock()
        .expect("GpioManager: lock should never fail");
    let mut ports = gpios.borrow_mut();

    match ports.iter().position(|e| e.gpio_number == number) {
        Some(n) => {
            let gpio = ports.remove(n);
            Some((gpio.initializer)(cfg))
        },
        None => None,
    }
}

pub(in crate::hal) fn bind_gpio(number: usize, initializer: fn(Config) -> GpioOutput) {
    let gpios = &GPIO_MANAGER
        .ports
        .try_lock()
        .expect("SerialManager: lock should never fail");
    let mut gpios = gpios.borrow_mut();
    if gpios
        .push(InitializerWrapper {
            gpio_number: number,
            initializer,
        })
        .is_err()
    {
        panic!("unable to bind gpio {number}")
    }
}

static GPIO_MANAGER: GpioManager = GpioManager {
    ports: Mutex::new(RefCell::new(Vec::new())),
};

struct InitializerWrapper {
    initializer: fn(Config) -> GpioOutput,
    gpio_number: usize,
}

struct GpioManager{
    ports: Mutex<CriticalSectionRawMutex, RefCell<Vec<InitializerWrapper, MAX_GPIO>>>,
}
