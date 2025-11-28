# Changelog

All notable changes to the BNO08x driver will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Full SPS v2.1.1 compliance documentation
- GitHub Actions CI/CD workflows (test, build, SBOM, release)
- SBOM generation and license policy validation scripts
- Comprehensive testing infrastructure with nextest support

### Changed

- Migrated repository from Bitbucket to GitHub (EdgeFirstAI/bno08x)
- Changed license from BSD-3-Clause to Apache-2.0
- Updated all source files with Apache-2.0 SPDX headers
- Modernized documentation (README, CONTRIBUTING, SECURITY, etc.)

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

[Unreleased]: https://github.com/EdgeFirstAI/bno08x/compare/v1.0.1...HEAD
[1.0.1]: https://github.com/EdgeFirstAI/bno08x/releases/tag/v1.0.1
