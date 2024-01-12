use crate::hal::serial;
use crate::libs::fmt;
use cortex_m::interrupt;
use embassy_stm32::{bind_interrupts, peripherals, time, usart, usb_otg};

bind_interrupts!(struct Irqs {
    UART7 => usart::InterruptHandler<peripherals::UART7>;
    OTG_FS => usb_otg::InterruptHandler<peripherals::USB_OTG_FS>;
});

fn config() -> embassy_stm32::Config {
    use embassy_stm32::rcc::*;
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(embassy_stm32::rcc::Hse {
        mode: embassy_stm32::rcc::HseMode::Oscillator,
        freq: time::mhz(8),
    });
    config.rcc.hsi48 = Some(embassy_stm32::rcc::Hsi48Config {
        sync_from_usb: true,
    }); // needed for USB
    config.rcc.hsi = None;
    config.rcc.ls = LsConfig {
        rtc: RtcClockSource::DISABLE,
        lsi: false,
        lse: None,
    };
    config.rcc.csi = true;
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL100,
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV8),
        divr: None,
    });
    config.rcc.pll2 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL45,
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV5),
        divr: Some(PllDiv::DIV1),
    });
    config.rcc.pll3 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV2,
        mul: PllMul::MUL72,
        divp: Some(PllDiv::DIV3),
        divq: Some(PllDiv::DIV6),
        divr: Some(PllDiv::DIV9),
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.d1c_pre = AHBPrescaler::DIV1;
    config.rcc.ahb_pre = AHBPrescaler::DIV2;
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config.rcc.apb3_pre = APBPrescaler::DIV2;
    config.rcc.apb4_pre = APBPrescaler::DIV2;
    config.rcc.voltage_scale = VoltageScale::Scale1;

    #[cfg(feature = "defmt")]
    {
        config.enable_debug_during_sleep = true;
    }
    #[cfg(not(feature = "defmt"))]
    {
        config.enable_debug_during_sleep = false;
    }
    config
}

impl super::GenericBoard for super::Board {
    fn init() {
        let _ = embassy_stm32::init(config());

        serial0_bind();
        serial1_bind();

        unsafe { interrupt::enable() };
    }
}

fn serial1_bind() {
    let sc1: &'static mut serial::SerialRingBuf =
        static_cell::make_static!(ringbuf::StaticRb::default());

    let sc2: &'static mut serial::SerialRingBuf =
        static_cell::make_static!(ringbuf::StaticRb::default());
    let (p1, c1) = sc1.split_ref();
    let (p2, c2) = sc2.split_ref();
    serial::bind_port(serial::Protocol::MavlinkV2, 1, c1, p1, c2, p2);
}

fn serial0_bind() {
    let sc1: &'static mut serial::SerialRingBuf =
        static_cell::make_static!(ringbuf::StaticRb::default());

    let sc2: &'static mut serial::SerialRingBuf =
        static_cell::make_static!(ringbuf::StaticRb::default());
    let (p1, c1) = sc1.split_ref();
    let (p2, c2) = sc2.split_ref();
    serial::bind_port(serial::Protocol::MavlinkV2, 0, c1, p1, c2, p2);
}

pub mod hw_tasks {
    use embassy_stm32::spi;
    use embassy_usb::driver::EndpointError;
    use embedded_io_async::Write;

    use super::*;

    #[allow(unused)]
    async fn port_read<T: usart::BasicInstance, R: usart::RxDma<T>>(
        mut port: usart::UartRx<'_, T, R>,
        mut rb_producer: serial::RingBufWriteRef,
        name: &str,
    ) {
        let mut buf = [0u8; 100];
        loop {
            let res = port
                .read_until_idle(&mut buf)
                .await
                .map_err(|e| fmt::debug!("{}, read err {}", name, e));
            if let Ok(v) = res {
                let amt = core::cmp::min(rb_producer.free_len(), v);
                rb_producer.push_slice(&buf[..amt]);
                if amt < v {
                    fmt::debug!("{}, RBfull. written {}, requested {}", name, amt, v)
                }
            }
        }
    }

    #[allow(unused)]
    async fn port_write<T: usart::BasicInstance, R: usart::TxDma<T>>(
        mut port: usart::UartTx<'_, T, R>,
        mut consumer: serial::RingBufReadRef,
        name: &str,
    ) {
        let mut buf = [0u8; 100];
        loop {
            if consumer.len() == 0 {
                embassy_time::Timer::after_micros(5).await;
                continue;
            }

            let amt = consumer.pop_slice(&mut buf);
            defmt::debug!("{} writing len bytes {}", name, amt);
            let _ = port
                .write_all(&buf[..amt])
                .await
                .map_err(|e| fmt::debug!("{}, write error {}", name, e));
        }
    }

    #[embassy_executor::task]
    pub async fn serial0_runner(_cfg: serial::Config) {
        let p = unsafe { embassy_stm32::Peripherals::steal() };
        let (mut consumer, mut producer) = fmt::unwrap!(serial::find_port_rb_ref(0));

        let name = "Serial0";

        // Create the driver, from the HAL.
        let mut ep_out_buffer = [0u8; 2048];
        let mut config = usb_otg::Config::default();
        config.vbus_detection = false;
        let driver = usb_otg::Driver::new_fs(
            p.USB_OTG_FS,
            Irqs,
            p.PA12,
            p.PA11,
            &mut ep_out_buffer,
            config,
        );
        // Create embassy-usb Config
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("41mo");
        config.product = Some("USB-serial");
        config.serial_number = Some("12345678");
        // Required for windows compatibility.
        // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
        config.device_class = 0xEF;
        config.device_sub_class = 0x02;
        config.device_protocol = 0x01;
        config.composite_with_iads = true;
        // Create embassy-usb DeviceBuilder using the driver and config.
        // It needs some buffers for building the descriptors.
        let mut device_descriptor = [0; 256];
        let mut config_descriptor = [0; 256];
        let mut bos_descriptor = [0; 256];
        let mut control_buf = [0; 64];
        let mut state = embassy_usb::class::cdc_acm::State::new();

        let mut builder = embassy_usb::Builder::new(
            driver,
            config,
            &mut device_descriptor,
            &mut config_descriptor,
            &mut bos_descriptor,
            &mut [], // no msos descriptors
            &mut control_buf,
        );

        // Create classes on the builder.
        let class = embassy_usb::class::cdc_acm::CdcAcmClass::new(&mut builder, &mut state, 64);

        // Build the builder.
        let mut usb = builder.build();

        let usb_runner = usb.run();

        let (mut tx, mut rx) = class.split();

        let read = async move {
            let mut buf = [0u8; 128];
            loop {
                rx.wait_connection().await;
                loop {
                    let l = match rx.read_packet(&mut buf).await {
                        Ok(l) => l,
                        Err(e) => match e {
                            EndpointError::BufferOverflow => panic!("{} buffer overflow", name),
                            EndpointError::Disabled => break,
                        },
                    };
                    let amt = core::cmp::min(producer.free_len(), l);
                    producer.push_slice(&buf[..amt]);
                    if amt < l {
                        fmt::debug!("{}, RBfull. written {}, requested {}", name, amt, l)
                    }
                }
            }
        };

        let write = async move {
            let mut buf = [0u8; 64];
            loop {
                tx.wait_connection().await;
                loop {
                    if consumer.is_full() {
                        consumer.clear();
                    }
                    if consumer.len() == 0 {
                        embassy_time::Timer::after_micros(5).await;
                        continue;
                    }
                    let ps = tx.max_packet_size();
                    let amt = consumer.pop_slice(&mut buf[..ps as usize]);

                    let r = tx.write_packet(&buf[..amt]).await;
                    if let Err(e) = r {
                        match e {
                            EndpointError::BufferOverflow => panic!("{} buffer overflow", name),
                            EndpointError::Disabled => break,
                        }
                    }
                }
            }
        };

        embassy_futures::join::join3(usb_runner, read, write).await;
    }

    #[embassy_executor::task]
    pub async fn serial1_runner(cfg: serial::Config) {
        let uc: usart::Config = cfg.into();

        let p = unsafe { embassy_stm32::Peripherals::steal() };
        let port = fmt::unwrap!(usart::Uart::new(
            p.UART7, p.PE7, p.PE8, Irqs, p.DMA1_CH0, p.DMA1_CH1, uc
        ));
        let (consumer, producer) = fmt::unwrap!(serial::find_port_rb_ref(1));

        let (tx, rx) = port.split();
        let name = "Serial1";

        let reader = port_read(rx, producer, name);
        let writer = port_write(tx, consumer, name);

        embassy_futures::join::join(reader, writer).await;
    }

    #[embassy_executor::task]
    pub async fn imu_task(
        mut rb: ringbuf::StaticProducer<
            'static,
            (nalgebra::Vector3<f32>, nalgebra::Vector3<f32>),
            5,
        >,
    ) {
        use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
        use embassy_sync::blocking_mutex::raw::NoopRawMutex;
        use embassy_sync::mutex::Mutex;

        let mut spi_config = spi::Config::default();
        spi_config.mode = spi::MODE_3;
        spi_config.bit_order = spi::BitOrder::MsbFirst;
        spi_config.frequency = time::mhz(8);
        let p = unsafe { embassy_stm32::Peripherals::steal() };

        let cs_pin = embassy_stm32::gpio::Output::new(
            p.PC15,
            embassy_stm32::gpio::Level::High,
            embassy_stm32::gpio::Speed::Low,
        );

        let spi = spi::Spi::new(
            p.SPI1, p.PA5, p.PD7, p.PA6, p.DMA1_CH5, p.DMA1_CH4, spi_config,
        );
        static SPI_BUS: static_cell::StaticCell<
            Mutex<
                NoopRawMutex,
                spi::Spi<peripherals::SPI1, peripherals::DMA1_CH5, peripherals::DMA1_CH4>,
            >,
        > = static_cell::StaticCell::new();

        let bus = SPI_BUS.init(Mutex::new(spi));
        let dev = SpiDevice::new(bus, cs_pin);
        let mut dev = icm42688::ICM42688::new(dev).await.unwrap();
        dev.fifo_reset().await.unwrap();
        dev.set_filter_and_scaling(embassy_time::Delay)
            .await
            .unwrap();
        const FIFO_BUFF_LEN: usize = 8;
        let mut buff = [icm42688::FIFOData::default(); FIFO_BUFF_LEN];

        let mut accumulated_accel = nalgebra::Vector3::<f32>::default();
        let mut accumulated_gyro = nalgebra::Vector3::<f32>::default();
        let mut accumulator: usize = 0;

        loop {
            let mut n_samples = dev.get_sample_num().await.unwrap();

            'outer: while n_samples > 0 {
                let n = core::cmp::min(n_samples as usize, FIFO_BUFF_LEN);
                dev.get_n_samples(&mut buff[..n]).await.unwrap();

                for i in buff[..n].iter() {
                    if !i.check_header() {
                        fmt::debug!("header err");
                        break 'outer;
                    }

                    let mut acc = nalgebra::Vector3::<f32>::new(
                        -1.0 * i.accel[1] as f32,
                        -1.0 * i.accel[0] as f32,
                        -1.0 * (i.accel[2] as f32),
                    );
                    let mut gyr = nalgebra::Vector3::<f32>::new(
                        -1.0 * i.gyro[1] as f32,
                        -1.0 * i.gyro[0] as f32,
                        -1.0 * (i.gyro[2] as f32),
                    );
                    icm42688::apply_scales(&mut acc, &mut gyr);
                    accumulated_accel += acc;
                    accumulated_gyro += gyr;
                    accumulator += 1;
                }
                n_samples -= n as u16;
            }

            if accumulator > 100 {
                accumulated_accel /= accumulator as f32;
                accumulated_gyro /= accumulator as f32;

                let _ = rb.push((accumulated_accel, accumulated_gyro));

                accumulator = 0;
                accumulated_accel *= 0.0;
                accumulated_gyro *= 0.0;
            }

            const DELAY: u64 = 1000000 / 1000;
            embassy_time::Timer::after_micros(DELAY).await;
        }
    }
}

#[cfg(not(feature = "defmt"))]
mod nondefmt {
    use core::panic::PanicInfo;
    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        loop {}
    }
}
