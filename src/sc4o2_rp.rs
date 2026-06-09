use circular_buffer::CircularBuffer;
use embassy_rp::uart;
use embassy_rp::uart::Error::Overrun;
use embassy_rp::uart::{Async, Uart};
use embassy_time::Duration;

use crate::{SC4O2Error, SC4O2Response};

const PAYLOAD_SIZE: usize = 9;
const BUFFER_SIZE: usize = PAYLOAD_SIZE * 3;
const FRAME_START: u8 = 0xFF;
const FRAME_COMMAND: u8 = 0x86;

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
            } else if let Some(response) = &self.last_response {
                Ok(response.clone())
            } else {
                Err(SC4O2Error::NoData)
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
                        log::info!("UART other error: {e:?}");
                        break;
                    }
                },
            }
        }

        let frame = self.next_frame()?;
        let o2 = u16::from_be_bytes([frame[2], frame[3]]) as f32 / 10.0;
        let response = SC4O2Response { o2 };
        self.last_response = Some(response.clone());
        self.last_read_time = Some(embassy_time::Instant::now());
        Ok(response)
    }

    fn next_frame(&mut self) -> Result<[u8; PAYLOAD_SIZE], SC4O2Error> {
        let mut saw_checksum_error = false;

        while self.circular_buffer.len() >= PAYLOAD_SIZE {
            if self.circular_buffer.nth_front(0).copied() != Some(FRAME_START) {
                self.circular_buffer.pop_front();
                continue;
            }

            if self.circular_buffer.nth_front(1).copied() != Some(FRAME_COMMAND) {
                self.circular_buffer.pop_front();
                continue;
            }

            let mut frame = [0u8; PAYLOAD_SIZE];
            for (index, byte) in frame.iter_mut().enumerate() {
                *byte = self.circular_buffer.nth_front(index).copied().unwrap();
            }

            let checksum = Self::checksum(&frame);
            if checksum == frame[8] {
                for _ in 0..PAYLOAD_SIZE {
                    self.circular_buffer.pop_front();
                }

                return Ok(frame);
            }

            log::error!("Checksum mismatch: expected {}, got {}", checksum, frame[8]);
            saw_checksum_error = true;
            self.circular_buffer.pop_front();
        }

        if saw_checksum_error {
            Err(SC4O2Error::ChecksumError)
        } else {
            Err(SC4O2Error::NoData)
        }
    }

    fn checksum(frame: &[u8; PAYLOAD_SIZE]) -> u8 {
        !(frame[0] as u16
            + frame[1] as u16
            + frame[2] as u16
            + frame[3] as u16
            + frame[4] as u16
            + frame[5] as u16
            + frame[6] as u16
            + frame[7] as u16) as u8
    }
}
