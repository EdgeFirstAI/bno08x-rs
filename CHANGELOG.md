# Changelog

All notable changes to the BNO08x driver will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [2.0.0] - Unreleased

### Breaking Changes

This release contains breaking API changes. See [Migration Guide](#migration-from-10x-to-200) below.

#### Renamed Types and Functions

| Old (v1.x) | New (v2.0) |
|------------|------------|
| `BNO08x::new_bno08x()` | `BNO08x::new_spi()` |
| `BNO08x::new_bno08x_from_symbol()` | `BNO08x::new_spi_from_symbol()` |
| `WrapperError<E>` | `DriverError<E>` |

#### Changed Import Paths

| Old (v1.x) | New (v2.0) |
|------------|------------|
| `bno08x::wrapper::BNO08x` | `bno08x::BNO08x` |
| `bno08x::wrapper::WrapperError` | `bno08x::DriverError` |
| `bno08x::wrapper::SENSOR_REPORTID_*` | `bno08x::SENSOR_REPORTID_*` |

### Added

- Full SPS v2.1.1 compliance documentation
- GitHub Actions CI/CD workflows (test, build, SBOM, release)
- SBOM generation and license policy validation scripts
- Comprehensive testing infrastructure with nextest support
- New modular code organization:
  - `src/driver.rs` - Main BNO08x driver implementation
  - `src/constants.rs` - Protocol constants and Q-point conversion functions
  - `src/reports.rs` - Sensor data structures and report parsing
  - `src/frs.rs` - Flash Record System helper functions

### Changed

- Migrated repository from Bitbucket to GitHub (EdgeFirstAI/bno08x)
- Changed license from BSD-3-Clause to Apache-2.0
- Updated all source files with Apache-2.0 SPDX headers
- Modernized documentation (README, CONTRIBUTING, SECURITY, etc.)
- Refactored monolithic `wrapper.rs` (~1400 lines) into focused modules
- Renamed "wrapper" terminology to "driver" throughout codebase
- Main types (`BNO08x`, `DriverError`, `SENSOR_REPORTID_*`) now re-exported at crate root

### Removed

- `bno08x-frs` binary (example code was misplaced as a binary target)
- `wrapper` module (replaced by `driver` module with cleaner API)

### Fixed

- Improved error handling in SPI interface (now returns errors instead of only logging)
- Fixed `geomag_rotation_quaternion()` returning wrong field (was returning `rotation_quaternion`)
- Fixed typo: `uncalib_gryo` renamed to `uncalib_gyro`

### Migration from 1.0.x to 2.0.0

#### Step 1: Update Import Statements

```rust
// Before (v1.x)
use bno08x::wrapper::{
    BNO08x, WrapperError,
    SENSOR_REPORTID_ACCELEROMETER, SENSOR_REPORTID_ROTATION_VECTOR,
};

// After (v2.0)
use bno08x::{
    BNO08x, DriverError,
    SENSOR_REPORTID_ACCELEROMETER, SENSOR_REPORTID_ROTATION_VECTOR,
};
```

#### Step 2: Update Constructor Calls

```rust
// Before (v1.x)
let mut imu = BNO08x::new_bno08x_from_symbol(
    "/dev/spidev1.0",
    "IMU_INT",
    "IMU_RST"
)?;

// After (v2.0)
let mut imu = BNO08x::new_spi_from_symbol(
    "/dev/spidev1.0",
    "IMU_INT",
    "IMU_RST"
)?;
```

Or if using explicit GPIO chip/pin numbers:

```rust
// Before (v1.x)
let mut imu = BNO08x::new_bno08x(
    "/dev/spidev1.0",
    "/dev/gpiochip0", 10,
    "/dev/gpiochip0", 11
)?;

// After (v2.0)
let mut imu = BNO08x::new_spi(
    "/dev/spidev1.0",
    "/dev/gpiochip0", 10,
    "/dev/gpiochip0", 11
)?;
```

#### Step 3: Update Error Type References

```rust
// Before (v1.x)
fn init_sensor() -> Result<(), WrapperError<SpiError>> {
    // ...
}

// After (v2.0)
fn init_sensor() -> Result<(), DriverError<SpiError>> {
    // ...
}
```

#### Unchanged APIs

The following APIs remain unchanged and require no migration:

- `init()`, `soft_reset()`
- `enable_report()`, `enable_rotation_vector()`, `enable_linear_accel()`, `enable_gyro()`, `enable_gravity()`
- `accelerometer()`, `rotation_quaternion()`, `game_rotation_quaternion()`, `geomag_rotation_quaternion()`
- `linear_accel()`, `gravity()`, `gyro()`, `gyro_uncalib()`, `mag_field()`
- `rotation_acc()`, `geomag_rotation_acc()`
- `handle_messages()`, `handle_all_messages()`, `handle_one_message()`, `eat_all_messages()`
- `set_sensor_orientation()`
- `add_sensor_report_callback()`, `remove_sensor_report_callback()`
- `is_report_enabled()`, `report_update_time()`
- `free()`

## [1.0.1] - 2023-11-27

### Added

- BNO08x userspace driver library with SPI interface
- Support for rotation vector quaternions
- Support for accelerometer, gyroscope, and magnetometer data
- Linear acceleration and gravity vector support
- Configurable sensor report rates
- Flash Record System (FRS) support for sensor orientation configuration
- Callback-based sensor event handling
- Two example binaries: `bno08x` and `bno08x-frs`
- GPIO control via gpiod library
- SPI communication via spidev
- Comprehensive sensor data structures and constants
- SHTP (Sensor Hub Transport Protocol) implementation
- Packet parsing and serialization
- Sensor initialization and reset logic

### Implementation Details

- **Core Components**:
  - `wrapper.rs`: Main BNO08x driver implementation with sensor fusion support
  - `interface/spi.rs`: SPI communication interface with GPIO control
  - `interface/spidev.rs`: Linux spidev wrapper
  - `interface/gpio.rs`: GPIO abstraction using gpiod
  - `interface/delay.rs`: Timing utilities

- **Sensor Capabilities**:
  - Rotation Vector (quaternion with heading accuracy)
  - Game Rotation Vector (quaternion without magnetometer)
  - Geomagnetic Rotation Vector (quaternion using magnetometer)
  - Accelerometer (calibrated, m/s²)
  - Gyroscope (calibrated and uncalibrated, rad/s)
  - Magnetometer (calibrated, µT)
  - Linear Acceleration (m/s²)
  - Gravity Vector (m/s²)

- **Protocol Support**:
  - SHTP packet structure (header + payload)
  - Multiple communication channels (command, executable, control, reports)
  - Sequence number tracking per channel
  - Product ID verification
  - Error reporting and handling
  - Feature enable/disable commands

[Unreleased]: https://github.com/EdgeFirstAI/bno08x/compare/v2.0.0...HEAD
[2.0.0]: https://github.com/EdgeFirstAI/bno08x/compare/v1.0.1...v2.0.0
[1.0.1]: https://github.com/EdgeFirstAI/bno08x/releases/tag/v1.0.1
