#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, I2C0, UART0, USB};
use embassy_rp::{bind_interrupts, dma, i2c, uart};
use embassy_rp::uart::Uart;
use embassy_rp::usb::Driver;
// use embassy_sc4o2_sensor::{SC4O2Error, SC4O2Sensor};
use embassy_time::{Duration, Timer};
use panic_probe as _;
use embassy_sc4o2_sensor::{SC4O2Error, SC4O2Sensor};

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
    UART0_IRQ => uart::InterruptHandler<UART0>;
    USBCTRL_IRQ =>  embassy_rp::usb::InterruptHandler<USB>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver).unwrap());


    let mut config = uart::Config::default();
    config.baudrate = 9600;
    config.data_bits = uart::DataBits::DataBits8;
    config.stop_bits = uart::StopBits::STOP1;
    config.parity = uart::Parity::ParityNone;

    let uart = Uart::new(
        p.UART0,
        p.PIN_12,
        p.PIN_13,
        Irqs,
        p.DMA_CH0,
        p.DMA_CH1,
        config,
    );
    let mut o2 = SC4O2Sensor::new(uart);

    // Read sensor data
    loop {
        match o2.read().await {
            Ok(data) => {
                log::info!(
                    "O2: {}",
                    data.o2
                );
            }
            Err(e) => match e {
                SC4O2Error::NoData => log::error!("No data"),
                SC4O2Error::ChecksumError => log::error!("Checksum error"),
            },
        }

        Timer::after(Duration::from_secs(1)).await;
    }
}


#[embassy_executor::task]
pub async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}
