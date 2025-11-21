#![no_std]
#![no_main]

mod sc4o2_rp;

pub use sc4o2_rp::SC4O2Sensor;

#[derive(Clone)]
pub struct SC4O2Response {
    pub o2: f32,
}

#[derive(Debug, Clone)]
pub enum SC4O2Error {
    ChecksumError,
    NoData,
}

