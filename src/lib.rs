// Copyright 2025 Au-Zone Technologies Inc.
// SPDX-License-Identifier: Apache-2.0

//! BNO08x IMU Driver
//!
//! A Rust userspace driver for the BNO08x family of IMU sensors.
//!
//! # Overview
//!
//! This crate provides a driver for communicating with Bosch/Hillcrest Labs
//! BNO08x IMU sensors over SPI. The driver handles the SHTP protocol and
//! provides high-level access to sensor data including:
//!
//! - Accelerometer
//! - Gyroscope (calibrated and uncalibrated)
//! - Magnetometer
//! - Rotation vectors (absolute, game, and geomagnetic)
//! - Linear acceleration (gravity removed)
//! - Gravity vector
//!
//! # Example
//!
//! ```no_run
//! use bno08x::{BNO08x, SENSOR_REPORTID_ACCELEROMETER};
//!
//! fn main() -> std::io::Result<()> {
//!     let mut imu = BNO08x::new_spi_from_symbol("/dev/spidev1.0", "IMU_INT", "IMU_RST")?;
//!
//!     imu.init().expect("Failed to initialize IMU");
//!     imu.enable_report(SENSOR_REPORTID_ACCELEROMETER, 100)
//!         .unwrap();
//!
//!     loop {
//!         imu.handle_all_messages(100);
//!         let accel = imu.accelerometer().unwrap();
//!         println!("Accel: {:?}", accel);
//!     }
//! }
//! ```

pub mod constants;
pub mod driver;
pub mod frs;
pub mod interface;
pub mod reports;

// Re-export main driver types at crate root for convenience
pub use constants::{
    SENSOR_REPORTID_ACCELEROMETER, SENSOR_REPORTID_GRAVITY, SENSOR_REPORTID_GYROSCOPE,
    SENSOR_REPORTID_GYROSCOPE_UNCALIB, SENSOR_REPORTID_LINEAR_ACCEL,
    SENSOR_REPORTID_MAGNETIC_FIELD, SENSOR_REPORTID_ROTATION_VECTOR,
    SENSOR_REPORTID_ROTATION_VECTOR_GAME, SENSOR_REPORTID_ROTATION_VECTOR_GEOMAGNETIC,
};
pub use driver::{BNO08x, DriverError};

/// Low-level errors from the communication interface
#[derive(Debug)]
pub enum Error<CommE, PinE> {
    /// Sensor communication error
    Comm(CommE),
    /// Pin setting error
    Pin(PinE),

    /// The sensor is not responding
    SensorUnresponsive,

    /// Buffer overflow - packet too large for receive buffer
    BufferOverflow {
        /// Size of the packet that was received
        packet_size: usize,
        /// Size of the buffer available
        buffer_size: usize,
    },

    /// No data available from sensor (timeout waiting for HINTN)
    NoDataAvailable,
}
