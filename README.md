# BNO08x IMU Driver

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build Status](https://github.com/EdgeFirstAI/bno08x/actions/workflows/build.yml/badge.svg)](https://github.com/EdgeFirstAI/bno08x/actions/workflows/build.yml)
[![Test Status](https://github.com/EdgeFirstAI/bno08x/actions/workflows/test.yml/badge.svg)](https://github.com/EdgeFirstAI/bno08x/actions/workflows/test.yml)

Rust userspace driver for the BNO08x family of 9-axis Inertial Measurement Units (IMUs) with sensor fusion.

## Overview

The BNO08x is a System-in-Package (SiP) that integrates a triaxial 14-bit accelerometer, a triaxial 16-bit gyroscope, a triaxial geomagnetic sensor, and a 32-bit microcontroller running SHTP (Sensor Hub Transport Protocol) firmware for sensor fusion. This library provides a safe Rust interface for communicating with the sensor over SPI and GPIO.

## Features

- SPI communication interface with GPIO control
- Sensor fusion quaternion output (rotation vectors)
- Raw sensor data access (accelerometer, gyroscope, magnetometer)
- Linear acceleration and gravity vectors
- Configurable report rates
- Sensor calibration support
- Callback-based sensor event handling
- Two example binaries: `bno08x` (full sensor demo) and `bno08x-frs` (sensor orientation configuration)

## Requirements

- Rust 1.70 or later
- Linux with GPIO and SPI support
- Physical BNO08x sensor connected via SPI

## Building

```bash
cargo build --release
```

## Running

### Full Sensor Demo

The `bno08x` binary demonstrates all sensor features:

```bash
cargo run --bin bno08x --release
```

### Sensor Orientation Configuration

The `bno08x-frs` binary sets sensor orientation using FRS (Flash Record System):

```bash
cargo run --bin bno08x-frs --release
```

## Library Usage

```rust
use bno08x::{
    interface::{
        gpio::{GpiodIn, GpiodOut},
        spidev::SpiDevice,
        SpiInterface,
    },
    wrapper::{BNO08x, SENSOR_REPORTID_ROTATION_VECTOR},
};

// Create driver instance
let mut imu = BNO08x::new_bno08x_from_symbol(
    "/dev/spidev1.0",
    "IMU_INT",
    "IMU_RST"
)?;

// Initialize the sensor
imu.init()?;

// Enable rotation vector reports at 100ms intervals
imu.enable_rotation_vector(100)?;

// Read quaternion data
let [qi, qj, qk, qr] = imu.rotation_quaternion()?;
```

## Testing

```bash
cargo test
```

Note: Integration tests require physical hardware and are skipped in CI. Run on target hardware with appropriate SPI/GPIO setup.

## Documentation

For detailed API documentation:

```bash
cargo doc --open
```

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on contributing to this project.

## Architecture

See [ARCHITECTURE.md](ARCHITECTURE.md) for detailed information about the driver architecture and design decisions.

## License

Copyright 2025 Au-Zone Technologies Inc.

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE) for details.

## Security

For security vulnerabilities, see [SECURITY.md](SECURITY.md).

## Related Projects

- [EdgeFirst Fusion](https://github.com/EdgeFirstAI/fusion) - Multi-modal sensor fusion for EdgeFirst Maivin platform
- [EdgeFirst Samples](https://github.com/EdgeFirstAI/samples) - Example applications using EdgeFirst components
