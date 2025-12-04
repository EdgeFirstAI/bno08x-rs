// Copyright 2025 Au-Zone Technologies Inc.
// SPDX-License-Identifier: Apache-2.0

//! # BNO08x IMU Driver
//!
//! A Rust userspace driver for the BNO08x family of 9-axis IMU sensors
//! from Bosch/Hillcrest Labs.
//!
//! ## Overview
//!
//! The BNO08x is a System-in-Package (SiP) that integrates:
//! - Triaxial 14-bit accelerometer
//! - Triaxial 16-bit gyroscope  
//! - Triaxial geomagnetic sensor
//! - 32-bit microcontroller running sensor fusion firmware
//!
//! This crate provides a safe Rust interface for communicating with the
//! sensor over SPI, handling the SHTP (Sensor Hub Transport Protocol) and
//! providing high-level access to fused and raw sensor data.
//!
//! ## Features
//!
//! - **Sensor Fusion**: Rotation vectors (absolute, game, geomagnetic)
//! - **Raw Sensors**: Accelerometer, gyroscope, magnetometer
//! - **Derived Data**: Linear acceleration, gravity vector
//! - **Configurable Rates**: 1 Hz to 1 kHz update rates
//! - **GPIO Integration**: Device tree symbolic name support for Linux
//! - **Callbacks**: Event-driven sensor data handling
//!
//! ## Quick Start
//!
//! ```no_run
//! use bno08x_rs::{BNO08x, SENSOR_REPORTID_ACCELEROMETER};
//!
//! fn main() -> std::io::Result<()> {
//!     // Create driver using GPIO symbolic names
//!     let mut imu = BNO08x::new_spi_from_symbol(
//!         "/dev/spidev1.0",  // SPI device
//!         "IMU_INT",         // Interrupt GPIO name
//!         "IMU_RST",         // Reset GPIO name
//!     )?;
//!
//!     // Initialize and configure
//!     imu.init().expect("Failed to initialize IMU");
//!     imu.enable_report(SENSOR_REPORTID_ACCELEROMETER, 100)?;  // 10 Hz
//!
//!     // Main loop
//!     loop {
//!         imu.handle_all_messages(100);
//!         let accel = imu.accelerometer()?;
//!         println!("Accel: {:?}", accel);
//!     }
//! }
//! ```
//!
//! ## Sensor Reports
//!
//! Enable specific sensor reports using their report ID constants:
//!
//! | Report | Constant | Method | Units |
//! |--------|----------|--------|-------|
//! | Accelerometer | [`SENSOR_REPORTID_ACCELEROMETER`] | [`accelerometer()`](BNO08x::accelerometer) | m/s² |
//! | Gyroscope | [`SENSOR_REPORTID_GYROSCOPE`] | [`gyro()`](BNO08x::gyro) | rad/s |
//! | Magnetometer | [`SENSOR_REPORTID_MAGNETIC_FIELD`] | [`mag_field()`](BNO08x::mag_field) | µT |
//! | Rotation Vector | [`SENSOR_REPORTID_ROTATION_VECTOR`] | [`rotation_quaternion()`](BNO08x::rotation_quaternion) | quaternion |
//! | Game Rotation | [`SENSOR_REPORTID_ROTATION_VECTOR_GAME`] | [`game_rotation_quaternion()`](BNO08x::game_rotation_quaternion) | quaternion |
//! | Geomagnetic Rotation | [`SENSOR_REPORTID_ROTATION_VECTOR_GEOMAGNETIC`] | [`geomag_rotation_quaternion()`](BNO08x::geomag_rotation_quaternion) | quaternion |
//! | Linear Acceleration | [`SENSOR_REPORTID_LINEAR_ACCEL`] | [`linear_accel()`](BNO08x::linear_accel) | m/s² |
//! | Gravity | [`SENSOR_REPORTID_GRAVITY`] | [`gravity()`](BNO08x::gravity) | m/s² |
//!
//! ## Hardware Requirements
//!
//! - Linux with SPI (`spidev`) and GPIO (`gpiod`) support
//! - BNO08x sensor connected via SPI
//! - GPIO for interrupt (HINTN) and reset (RSTN) signals
//!
//! ## More Information
//!
//! - [Repository](https://github.com/EdgeFirstAI/bno08x-rs)
//! - [crates.io](https://crates.io/crates/bno08x-rs)
//! - [Maivin Platform](https://www.edgefirst.ai/edgefirstmodules)

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
