//! Shared sensor configuration and module enables for specific impl

// Image dimensions
pub const H_RES: usize = 160;
pub const V_RES: usize = 128;
pub const IMAGE_SCALE: usize = 1;

#[cfg(feature = "ov5640")]
pub mod ov5640;
#[cfg(feature = "ov5640")]
pub use ov5640 as sensor;
