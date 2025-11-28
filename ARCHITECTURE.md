# Architecture

This document describes the architecture and design decisions of the BNO08x IMU driver.

## Overview

The BNO08x driver is a Rust userspace library that provides safe, ergonomic access to the BNO08x family of 9-axis Inertial Measurement Units (IMUs). The driver communicates with the sensor over SPI and uses GPIO for reset and interrupt handling.

## Design Principles

1. **Safety First**: Use Rust's type system to prevent common errors (buffer overflows, race conditions, etc.)
2. **Hardware Abstraction**: Provide clean abstractions that work across different hardware platforms
3. **Performance**: Minimize overhead and avoid unnecessary allocations in hot paths
4. **Testability**: Design components to be testable without physical hardware where possible
5. **Ergonomics**: Provide intuitive APIs that make common tasks easy

## Component Architecture

```text
┌─────────────────────────────────────────────────────────┐
│                   Application Layer                      │
│              (bno08x binary, user code)                  │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│                  BNO08x Wrapper                          │
│                  (wrapper.rs)                            │
│  - Sensor initialization and configuration               │
│  - Report management (enable/disable sensors)            │
│  - Data conversion (Q-point to float)                    │
│  - Callback registration and dispatch                    │
│  - FRS (Flash Record System) operations                  │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              SensorInterface Trait                       │
│              (interface/mod.rs)                          │
│  - Abstract interface for sensor communication           │
│  - Send/receive packets                                  │
│  - Timeout handling                                      │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│                SpiInterface Implementation               │
│                (interface/spi.rs)                        │
│  - SPI packet send/receive                               │
│  - GPIO interrupt (HINTN) monitoring                     │
│  - GPIO reset control (RSTN)                             │
│  - Packet header parsing                                 │
└───────────┬────────────────────────────────┬────────────┘
            │                                │
            ▼                                ▼
┌────────────────────────┐      ┌──────────────────────────┐
│    SpiDevice Wrapper    │      │    GPIO Abstraction      │
│   (interface/spidev.rs) │      │  (interface/gpio.rs)     │
│  - Linux spidev API     │      │  - gpiod library wrapper │
│  - SPI transfer/write   │      │  - Input/Output pins     │
└────────────────────────┘      └──────────────────────────┘
```

## Module Breakdown

### Core Modules

#### `lib.rs`

- Entry point for the library
- Defines top-level error types
- Re-exports public interfaces

#### `wrapper.rs`

The main driver implementation containing:

- **`BNO08x<SI>` struct**: Generic over `SensorInterface` type
  - State management (sequence numbers, buffers, sensor data)
  - Initialization and reset logic
  - Report enable/disable commands
  - Data accessors (quaternions, acceleration, etc.)
  - Callback management for sensor events

- **SHTP Protocol Implementation**:
  - Packet encoding/decoding
  - Channel management (command, executable, control, reports)
  - Sequence number tracking
  - Error handling

- **Sensor Data Processing**:
  - Q-point fixed-point to float conversion
  - Report ID handling for different sensor types
  - Timestamp tracking for each report type

### Interface Layer

#### `interface/mod.rs`

Defines the `SensorInterface` trait:

```rust
pub trait SensorInterface {
    type SensorError;
    fn setup(&mut self) -> Result<(), Self::SensorError>;
    fn write_packet(&mut self, packet: &[u8]) -> Result<(), Self::SensorError>;
    fn read_packet(&mut self, recv_buf: &mut [u8]) -> Result<usize, Self::SensorError>;
    fn read_with_timeout(&mut self, recv_buf: &mut [u8], max_ms: usize) -> Result<usize, Self::SensorError>;
    fn send_and_receive_packet(&mut self, send_buf: &[u8], recv_buf: &mut [u8]) -> Result<usize, Self::SensorError>;
    fn requires_soft_reset(&self) -> bool;
}
```

This abstraction allows for different communication backends (SPI, I2C, etc.) without changing the core driver logic.

#### `interface/spi.rs`

SPI-specific implementation:

- **`SpiControlLines<SPI, IN, RSTN>`**: Encapsulates SPI device + GPIO pins
- **`SpiInterface<SPI, IN, RSTN>`**: Implements `SensorInterface` for SPI
  - Hardware interrupt monitoring (HINTN pin)
  - Reset sequencing
  - Packet transfer with proper timing
  - Error recovery

#### `interface/spidev.rs`

Linux spidev wrapper:

- **`SpiDevice`**: Wraps Linux `spidev` for SPI communication
- Implements `Transfer` and `Write` traits
- Configures SPI mode, speed, bit order

#### `interface/gpio.rs`

GPIO abstraction using gpiod:

- **`GpiodIn`**: Input pin implementation (HINTN)
- **`GpiodOut`**: Output pin implementation (RSTN)
- Implements `InputPin` and `OutputPin` traits

#### `interface/delay.rs`

Timing utilities:

- `delay_ms()`: Blocking millisecond delay using `std::thread::sleep`

## Protocol: SHTP (Sensor Hub Transport Protocol)

### Packet Structure

```text
┌─────────────────────────────────────────────────────────┐
│                     Packet Header                        │
│  Bytes 0-1: Length (LSB, MSB with continuation flag)    │
│  Byte 2:    Channel number                               │
│  Byte 3:    Sequence number                              │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│                     Packet Body                          │
│  Byte 0:    Report ID or command                         │
│  Bytes 1+:  Report/command-specific data                 │
└─────────────────────────────────────────────────────────┘
```

### Communication Channels

- **Channel 0 (COMMAND)**: SHTP control commands and responses
- **Channel 1 (EXECUTABLE)**: Device-level commands (reset)
- **Channel 2 (HUB_CONTROL)**: Sensor hub control (enable reports, get product ID)
- **Channel 3 (SENSOR_REPORTS)**: Sensor data input reports

### Sensor Reports

The driver supports multiple report types, each with a unique ID:

| Report ID | Sensor Type                  | Q-Point Format |
|-----------|------------------------------|----------------|
| 0x01      | Accelerometer                | Q8             |
| 0x02      | Gyroscope (calibrated)       | Q9             |
| 0x03      | Magnetic Field (calibrated)  | Q4             |
| 0x04      | Linear Acceleration          | Q8             |
| 0x05      | Rotation Vector              | Q14 (quat), Q12 (acc) |
| 0x06      | Gravity                      | Q8             |
| 0x07      | Gyroscope (uncalibrated)     | Q9             |
| 0x08      | Game Rotation Vector         | Q14            |
| 0x09      | Geomagnetic Rotation Vector  | Q14 (quat), Q12 (acc) |

### Data Conversion

Sensor data is transmitted in Q-point fixed-point format. The driver converts to `f32`:

```rust
fn q_to_f32(q_val: i16, q_point: usize) -> f32 {
    (q_val as f32) / (1 << q_point) as f32
}
```

## Initialization Sequence

1. **Hardware Reset**: Toggle RSTN pin (low → high)
2. **Wait for HINTN**: Sensor asserts interrupt when ready
3. **Read Advertisement**: Sensor sends unsolicited advertisement packet
4. **Soft Reset** (optional): Send reset command for some interfaces
5. **Verify Product ID**: Request and validate sensor product ID
6. **Enable Reports**: Configure desired sensor reports with update rates

## Callback System

The driver supports registering callbacks for sensor events:

```rust
imu.add_sensor_report_callback(
    SENSOR_REPORTID_ROTATION_VECTOR,
    "my_callback".to_string(),
    |imu| {
        let quat = imu.rotation_quaternion().unwrap();
        println!("Quaternion: {:?}", quat);
    }
);
```

Callbacks are stored in a `HashMap` per report type and invoked when new data arrives.

## Thread Safety

**Note**: The current implementation is **NOT thread-safe**. The `BNO08x` struct does not implement `Send` or `Sync` due to GPIO/SPI handles. For multi-threaded use, wrap the driver in a `Mutex` or `Arc<Mutex<_>>`.

## Error Handling

Errors are propagated using `Result` types:

- **`Error<CommE, PinE>`**: Library-level errors
  - `Comm(CommE)`: Communication errors
  - `Pin(PinE)`: GPIO errors
  - `SensorUnresponsive`: Sensor did not respond after reset

- **`WrapperError<E>`**: Higher-level driver errors
  - `CommError(E)`: Sensor interface errors
  - `InvalidChipId(u8)`: Product ID mismatch
  - `InvalidFWVersion(u8)`: Unsupported firmware
  - `NoDataAvailable`: Expected data not received

## Performance Considerations

### Buffer Sizes

- **Send buffer**: 256 bytes (sufficient for all commands)
- **Receive buffer**: 2048 bytes (handles largest packets)

### Timing

- **SPI speed**: 80 kHz (configured for reliable communication)
- **Report rates**: Configurable from 1 Hz to 1 kHz (gyro-limited)
- **Polling interval**: Recommended 50ms for main loop

### Memory Allocation

- All buffers are stack-allocated (no heap allocations in hot path)
- Callbacks use boxed closures (one-time allocation at registration)

## Future Enhancements

Potential areas for improvement:

1. **Async/Await Support**: Non-blocking I/O using `tokio` or `async-std`
2. **I2C Interface**: Support for I2C communication backend
3. **DMA**: Direct memory access for faster SPI transfers
4. **Calibration Persistence**: Save/load calibration data from flash
5. **Tare Function**: Zero orientation at runtime
6. **External Sensors**: Support for barometer, light sensor integration

## References

- [BNO08x Datasheet](https://www.ceva-dsp.com/product/bno080-085/)
- [SHTP Reference Manual](https://www.hillcrestlabs.com/download/7316/)
- [Rust Embedded HAL](https://github.com/rust-embedded/embedded-hal)
- [gpiod Documentation](https://docs.rs/gpiod/latest/gpiod/)
- [spidev Documentation](https://docs.rs/spidev/latest/spidev/)

---

For implementation questions, see [CONTRIBUTING.md](CONTRIBUTING.md) or open a [GitHub Discussion](https://github.com/EdgeFirstAI/bno08x/discussions).
