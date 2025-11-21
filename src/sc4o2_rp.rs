use crate::{SC4O2Error, SC4O2Response};
use circular_buffer::CircularBuffer;
use embassy_rp::uart::Error::{Overrun};
use embassy_rp::uart::{Async, Uart};
use embassy_rp::{uart};
use embassy_time::{Duration};


const PAYLOAD_SIZE: usize = 9;
const BUFFER_SIZE: usize = PAYLOAD_SIZE * 3;

pub struct SC4O2Sensor<'a> {
    uart: Uart<'a, Async>,
    circular_buffer: CircularBuffer<BUFFER_SIZE, u8>,
    last_response: Option<SC4O2Response>,
    last_read_time: Option<embassy_time::Instant>,
    __marker: core::marker::PhantomData<&'a ()>,
}


impl<'a> SC4O2Sensor<'a> {
    pub fn new(uart: Uart<'a, Async>) -> Self {
        let mut config = uart::Config::default();
        config.baudrate = 9600;
        config.data_bits = uart::DataBits::DataBits8;
        config.stop_bits = uart::StopBits::STOP1;
        config.parity = uart::Parity::ParityNone;

        SC4O2Sensor {
            uart,
            circular_buffer: CircularBuffer::new(),
            last_response: None,
            last_read_time: None,
            __marker: core::marker::PhantomData,
        }
    }

    pub async fn read(&mut self) -> Result<SC4O2Response, SC4O2Error> {
        const MIN_READ_TIMEOUT: Duration = Duration::from_millis(500);
        let now = embassy_time::Instant::now();
        if let Some(last_read_time) = self.last_read_time {
            if now.duration_since(last_read_time) >= MIN_READ_TIMEOUT {
                self.read_internal().await
            } else {
                if let Some(response) = &self.last_response {
                    Ok(response.clone())
                } else {
                    Err(SC4O2Error::NoData)
                }
            }
        } else {
            self.last_read_time = Some(now);
            Err(SC4O2Error::NoData)
        }
    }

    async fn read_internal(&mut self) -> Result<SC4O2Response, SC4O2Error> {
        let mut max_attempts = 10;
        while max_attempts > 0 {
            let mut buf = [0u8; PAYLOAD_SIZE];
            match self.uart.read(&mut buf).await {
                Ok(_) => {
                    self.circular_buffer.extend_from_slice(&buf);
                    break;
                }
                Err(e) => match e {
                    Overrun => {
                        log::info!("UART overrun error");
                        max_attempts -= 1;
                        continue;
                    }
                    _ => {
                        log::info!("UART other error: {:?}", e);
                        break;
                    }
                },
            }
        }

        let mut ready_buf = [0u8; PAYLOAD_SIZE];
        let mut index = 0;
        let mut buffer_index = 0;
        while let Some(byte) = self.circular_buffer.nth_front(buffer_index) {
            buffer_index += 1;
            let byte = *byte;
            if byte == 0xFF && index == 0 {
                ready_buf[index] = byte;
                index += 1;
                continue;
            }
            else if byte == 0x86 && index == 1 {
                ready_buf[index] = byte;
                index += 1;
                continue;
            } else if index > 1 {
                ready_buf[index] = byte;
                index += 1;
                if index >= PAYLOAD_SIZE {
                    break;
                }
            }
        }

        let checksum = !(ready_buf[0] as u16
            + ready_buf[1] as u16
            + ready_buf[2] as u16
            + ready_buf[3] as u16
            + ready_buf[4] as u16
            + ready_buf[5] as u16
            + ready_buf[6] as u16
            + ready_buf[7] as u16) as u8;

        if checksum == ready_buf[8] {
            let o2 = u16::from_be_bytes([ready_buf[2], ready_buf[3]]) as f32 / 10.0;
            let response = SC4O2Response { o2 };
            self.last_response = Some(response.clone());
            self.last_read_time = Some(embassy_time::Instant::now());
            Ok(response)
        } else {
            log::error!("Checksum mismatch: expected {}, got {}", checksum, ready_buf[7]);
            Err(SC4O2Error::ChecksumError)
        }
    }
}
