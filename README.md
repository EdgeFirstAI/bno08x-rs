# BNO08x IMU Driver

[![Crates.io](https://img.shields.io/crates/v/bno08x-rs.svg)](https://crates.io/crates/bno08x-rs)
[![Documentation](https://docs.rs/bno08x-rs/badge.svg)](https://docs.rs/bno08x-rs)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build Status](https://github.com/EdgeFirstAI/bno08x-rs/actions/workflows/build.yml/badge.svg)](https://github.com/EdgeFirstAI/bno08x-rs/actions/workflows/build.yml)
[![Test Status](https://github.com/EdgeFirstAI/bno08x-rs/actions/workflows/test.yml/badge.svg)](https://github.com/EdgeFirstAI/bno08x-rs/actions/workflows/test.yml)
[![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=EdgeFirstAI_bno08x-rs&metric=alert_status)](https://sonarcloud.io/summary/new_code?id=EdgeFirstAI_bno08x-rs)
[![Coverage](https://sonarcloud.io/api/project_badges/measure?project=EdgeFirstAI_bno08x-rs&metric=coverage)](https://sonarcloud.io/summary/new_code?id=EdgeFirstAI_bno08x-rs)

Rust userspace driver for the BNO08x family of 9-axis Inertial Measurement Units (IMUs) with sensor fusion.

## Overview

The BNO08x is a System-in-Package (SiP) that integrates a triaxial 14-bit accelerometer, a triaxial 16-bit gyroscope, a triaxial geomagnetic sensor, and a 32-bit microcontroller running SHTP (Sensor Hub Transport Protocol) firmware for sensor fusion. This library provides a safe Rust interface for communicating with the sensor over SPI and GPIO.

## Features

- SPI communication with GPIO interrupt and reset control
- Sensor fusion quaternion output (rotation vectors: absolute, game, geomagnetic)
- Raw sensor data access (accelerometer, gyroscope, magnetometer)
- Linear acceleration and gravity vectors
- Configurable report rates (1 Hz to 1 kHz)
- Flash Record System (FRS) for sensor orientation configuration
- Callback-based sensor event handling
- Portable GPIO configuration via device tree symbolic names

## Requirements

- Rust 1.90 or later
- Linux with SPI (`spidev`) and GPIO (`gpiod`) support
- BNO08x sensor connected via SPI with GPIO for interrupt (HINTN) and reset (RSTN)

### Hardware Setup

The driver requires three connections to the BNO08x:

| Signal | Type | Description |
|--------|------|-------------|
| SPI | Bus | Data communication (MOSI, MISO, SCLK, CS) |
| HINTN | GPIO Input | Hardware interrupt - sensor signals data ready |
| RSTN | GPIO Output | Reset control - toggle low to reset sensor |

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
bno08x-rs = "2.0"
```

Or clone and build from source:

```bash
git clone https://github.com/EdgeFirstAI/bno08x-rs.git
cd bno08x-rs
cargo build --release
```

## Quick Start

### On Maivin Hardware

The [Maivin platform](https://edgefirst.ai/maivin/) has pre-configured device tree settings for the IMU:

```bash
# Run the demo binary
cargo run --release

# Or use as a library in your application
```

### Library Usage

```rust
use bno08x_rs::{BNO08x, SENSOR_REPORTID_ROTATION_VECTOR, SENSOR_REPORTID_ACCELEROMETER};
use bno08x_rs::interface::delay::delay_ms;

fn main() -> std::io::Result<()> {
    // Create driver using GPIO symbolic names (recommended)
    let mut imu = BNO08x::new_spi_from_symbol(
        "/dev/spidev1.0",  // SPI device
        "IMU_INT",         // Interrupt GPIO name
        "IMU_RST",         // Reset GPIO name
    )?;

    // Initialize the sensor
    imu.init().expect("Failed to initialize IMU");

    // Enable sensor reports with update intervals (milliseconds)
    imu.enable_report(SENSOR_REPORTID_ROTATION_VECTOR, 100)?;  // 10 Hz
    imu.enable_report(SENSOR_REPORTID_ACCELEROMETER, 50)?;     // 20 Hz

    // Main loop
    loop {
        // Process incoming sensor messages
        imu.handle_messages(10, 20);  // 10ms timeout, max 20 messages

        // Read sensor data
        let [qi, qj, qk, qr] = imu.rotation_quaternion()?;
        let [ax, ay, az] = imu.accelerometer()?;
        
        println!("Quaternion: [{:.3}, {:.3}, {:.3}, {:.3}]", qi, qj, qk, qr);
        println!("Accel: [{:.3}, {:.3}, {:.3}] m/s²", ax, ay, az);
        
        delay_ms(50);
    }
}
```

### Using Callbacks

```rust
use bno08x_rs::{BNO08x, SENSOR_REPORTID_ROTATION_VECTOR};

let mut imu = BNO08x::new_spi_from_symbol("/dev/spidev1.0", "IMU_INT", "IMU_RST")?;
imu.init()?;
imu.enable_report(SENSOR_REPORTID_ROTATION_VECTOR, 100)?;

// Register callback for rotation vector updates
imu.add_sensor_report_callback(
    SENSOR_REPORTID_ROTATION_VECTOR,
    "my_handler".to_string(),
    |imu| {
        let quat = imu.rotation_quaternion().unwrap();
        println!("Quaternion: {:?}", quat);
    },
);

loop {
    imu.handle_messages(10, 20);  // Callbacks invoked automatically
}
```

### Alternative: Explicit GPIO Pins

If your platform doesn't have named GPIO lines, specify chip and pin numbers directly:

```rust
let mut imu = BNO08x::new_spi(
    "/dev/spidev1.0",
    "/dev/gpiochip4", 31,  // HINTN: chip 4, pin 31
    "/dev/gpiochip5", 1,   // RSTN: chip 5, pin 1
)?;
```

## Testing

### Unit Tests

Run the unit tests (no hardware required):

```bash
cargo test
```

### Hardware Integration Tests

Integration tests require a physical BNO08x sensor and are marked with `#[ignore]`. Run them on target hardware:

```bash
# Run all hardware tests (single-threaded to avoid SPI conflicts)
RUST_LOG=debug cargo test -- --ignored --test-threads=1

# Run a specific test
RUST_LOG=debug cargo test test_accelerometer -- --ignored
```

### Testing on Maivin

The [Maivin platform](https://edgefirst.ai/maivin/) comes pre-configured with the correct device tree settings. Simply run:

```bash
cargo test -- --ignored --test-threads=1
```

The tests use these default settings:

- SPI device: `/dev/spidev1.0`
- Interrupt GPIO: `IMU_INT`
- Reset GPIO: `IMU_RST`

### Testing on Custom Hardware

For custom hardware platforms, you need to:

1. **Connect the BNO08x sensor** via SPI with HINTN and RSTN GPIO lines
2. **Configure the Linux device tree** with `gpio-line-names` for your GPIO controllers
3. **Verify GPIO names** are accessible:

   ```bash
   gpioinfo | grep -E "IMU_INT|IMU_RST"
   ```

4. **Update test constants** in `tests/hardware_integration.rs` if using different names:

   ```rust
   const TEST_SPI_DEVICE: &str = "/dev/spidev1.0";
   const TEST_INT_GPIO: &str = "IMU_INT";
   const TEST_RST_GPIO: &str = "IMU_RST";
   ```

See [ARCHITECTURE.md](ARCHITECTURE.md) for detailed GPIO configuration guidance.

## Running the Demo

The included binary demonstrates all sensor features:

```bash
cargo run --release
```

Output:

```text
Report 5 is enabled
Report 1 is enabled
Report 2 is enabled
Report 3 is enabled
loop_interval: 50
Attitude [degrees]: yaw=45.123, pitch=2.456, roll=-0.789, accuracy=0.052
Accelerometer [m/s^2]: ax=0.123, ay=-0.456, az=9.789
Gyroscope [rad/s]: gx=0.001, gy=-0.002, gz=0.000
Magnetometer [uTesla]: mx=25.123, my=-12.456, mz=42.789
timestamp [ns]: 1234567890123456789
```

## Documentation

API documentation is available on [docs.rs](https://docs.rs/bno08x-rs).

To build documentation locally:

```bash
cargo doc --open
```

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on contributing to this project.

## Architecture

See [ARCHITECTURE.md](ARCHITECTURE.md) for detailed information about:

- Driver architecture and design decisions
- Protocol stack (SH-2 and SHTP)
- GPIO pin mapping and device tree configuration
- Module structure and dependencies

## License

Copyright 2025 Au-Zone Technologies Inc.

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE) for details.

## Security

For security vulnerabilities, see [SECURITY.md](SECURITY.md).

## Related Projects

- [Maivin Platform](https://edgefirst.ai/maivin/) - EdgeFirst AI edge computing platform
- [Maivin Overlays](https://github.com/MaivinAI/maivin-overlays) - Device tree overlays for Maivin hardware
- [EdgeFirst Fusion](https://github.com/EdgeFirstAI/fusion) - Multi-modal sensor fusion
- [EdgeFirst Samples](https://github.com/EdgeFirstAI/samples) - Example applications
