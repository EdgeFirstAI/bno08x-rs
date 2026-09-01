# Changelog

All notable changes to the BNO08x driver will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [3.0.0] - 2026-09-01

Fixes the BNO08x SPI transport, which could silently lose every command sent
to a sleeping sensor hub and starve the hub's own report processing. On the
Maivin platform this showed up as `edgefirst-imu` failing to initialise with
`InvalidChipId(0)` and restarting roughly 2.5 times a minute; after this
change the same device ran a ten-minute soak with zero restarts.

### Breaking

- `init()` on SPI no longer verifies the product ID before returning. The hub
  sleeps as soon as its output queue is empty and, without a WAKE pin, cannot
  be woken again, so `init()` now leaves the product ID responses pending for
  the caller's first write instead of consuming them. Two consequences for
  callers:
  - **Enable the first report immediately after `init()` returns**, with no
    sleep in between. A caller that waits will find the hub asleep and its
    write rejected with `NoDataAvailable`.
  - `init()` no longer returns `InvalidChipId` on SPI. Call the new
    `product_id_verified()` after enabling the first report to confirm the
    sensor identified itself.
- The default SPI clock is raised from 80 kHz to 1 MHz. The BNO08x supports up
  to 3 MHz, but wiring that only tolerated 80 kHz will need
  `SpiDevice::new_with_speed()` to restore the old rate.
- A write to a sensor that is not signalling H_INTN now fails with
  `NoDataAvailable` rather than appearing to succeed. Code that ignored write
  results will start seeing errors it previously never saw — those writes were
  being discarded by the hardware all along.

### Added

- `BNO08x::product_id_verified()` reports whether a product ID response has
  been received since the last reset.
- `SpiDevice::new_with_speed()` and `DEFAULT_SPI_SPEED_HZ` for callers that
  need a different SPI clock.

### Fixed

- SPI writes are now gated on H_INTN so commands are no longer silently lost
  when the sensor hub sleeps after reset (no WAKE pin on boards that strap PS0
  high). A write on an idle sensor returns `NoDataAvailable` instead of
  pretending success (EDGEAI-1100)
- `init()` on SPI no longer drains the startup burst with fixed 250 ms sleeps
  and a trailing 100 ms sleep; it lets the hub queue its startup packets, then
  piggybacks product ID requests on the startup reads so that responses are
  still pending when the caller enables its first report (EDGEAI-1100)
- `init()` on SPI now waits for the hub to report both reset-complete and
  initialize before returning; sensor commands written earlier were ignored by
  the hub (EDGEAI-1100)
- After a reset the driver waits for the sensor to drive H_INTN high before
  waiting for it to assert. On a board whose pad defaults to a pull-down the
  undriven line read as asserted while the sensor was still booting
  (EDGEAI-1100)
- Packet reads wait for the H_INTN continuation instead of a fixed 5 ms sleep,
  and H_INTN is polled every 100 µs instead of every 1 ms, keeping the host
  inside the BNO085/086 timing budget so rotation vector output is no longer
  starved (EDGEAI-1100)
- Sensor hub control packets are now parsed report by report: the hub batches
  pending control reports (for example the initialize response together with
  the product ID responses, or a product ID response together with a
  get-feature response) into one SHTP packet, and only the first report used
  to be seen, so `enable_report()` could miss its own acknowledgement
  (EDGEAI-1100)
- `is_report_enabled()` off-by-one bounds check

### Changed

- `SpiDevice` reuses a fixed receive buffer instead of allocating on every
  transfer.
- Removed the 1 ms sleeps between messages in `handle_messages()`,
  `handle_all_messages()` and `eat_all_messages()`, and the 200 ms sleep at
  the end of `enable_report()`.

## [2.0.1] - 2025-12-22

### Changed

- Moved demo binary to examples directory
- Refactored `new_spi_from_symbol()` to reduce cognitive complexity
- Refactored `set_sensor_orientation()` to reduce cognitive complexity

### Fixed

- Improved error handling and fixed format/linting errors

### Documentation

- Added docs.rs links and crates.io badges to README

### Testing

- Improved unit test coverage for driver and GPIO modules
- Improved test coverage with packet handling and error tests

## [2.0.0] - 2025-12-04

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
| `bno08x::wrapper::BNO08x` | `bno08x_rs::BNO08x` |
| `bno08x::wrapper::WrapperError` | `bno08x_rs::DriverError` |
| `bno08x::wrapper::SENSOR_REPORTID_*` | `bno08x_rs::SENSOR_REPORTID_*` |

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
use bno08x_rs::{
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

[Unreleased]: https://github.com/EdgeFirstAI/bno08x-rs/compare/v3.0.0...HEAD
[3.0.0]: https://github.com/EdgeFirstAI/bno08x-rs/compare/v2.0.1...v3.0.0
[2.0.1]: https://github.com/EdgeFirstAI/bno08x-rs/compare/v2.0.0...v2.0.1
[2.0.0]: https://github.com/EdgeFirstAI/bno08x-rs/compare/v1.0.1...v2.0.0
[1.0.1]: https://github.com/EdgeFirstAI/bno08x-rs/releases/tag/v1.0.1
