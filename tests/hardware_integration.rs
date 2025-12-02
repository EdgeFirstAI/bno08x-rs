// Copyright 2025 Au-Zone Technologies Inc.
// SPDX-License-Identifier: Apache-2.0

//! Hardware integration tests for BNO08x IMU driver
//!
//! These tests require real hardware and are marked with #[ignore].
//! Run with: RUST_LOG=debug cargo test -- --ignored --test-threads=1
//!
//! Note: Tests use handle_one_message() or handle_messages(timeout, max_count)
//! instead of handle_all_messages() which loops forever when reports are
//! streaming.

use bno08x::{BNO08x, SENSOR_REPORTID_ACCELEROMETER, SENSOR_REPORTID_MAGNETIC_FIELD};
use std::{
    sync::{Arc, Mutex, Once},
    thread::sleep,
    time::Duration,
};

static INIT: Once = Once::new();

/// Initialize logger for tests (only once)
fn init_logger() {
    INIT.call_once(|| {
        env_logger::init();
    });
}

const TEST_SPI_DEVICE: &str = "/dev/spidev1.0";
const TEST_INT_GPIO: &str = "IMU_INT";
const TEST_RST_GPIO: &str = "IMU_RST";
const REPORT_INTERVAL_MS: u16 = 100;
const SENSOR_WARMUP_MS: u64 = 500;

// =============================================================================
// Basic Tests
// =============================================================================

#[test]
#[ignore]
fn test_imu_initialization() {
    init_logger();

    let mut imu = BNO08x::new_spi_from_symbol(TEST_SPI_DEVICE, TEST_INT_GPIO, TEST_RST_GPIO)
        .expect("Failed to create IMU driver");

    imu.init().expect("Failed to initialize IMU");
    sleep(Duration::from_millis(SENSOR_WARMUP_MS));

    println!("✓ IMU initialized successfully");
}

#[test]
#[ignore]
fn test_soft_reset() {
    init_logger();

    let mut imu = BNO08x::new_spi_from_symbol(TEST_SPI_DEVICE, TEST_INT_GPIO, TEST_RST_GPIO)
        .expect("Failed to create IMU driver");

    imu.init().expect("Failed to initialize IMU");
    sleep(Duration::from_millis(SENSOR_WARMUP_MS));

    imu.soft_reset().expect("Failed to perform soft reset");
    sleep(Duration::from_millis(SENSOR_WARMUP_MS * 2));

    imu.init().expect("Failed to re-initialize after reset");
    sleep(Duration::from_millis(SENSOR_WARMUP_MS));

    println!("✓ Soft reset successful");
}

// =============================================================================
// Sensor Reading Tests (using handle_one_message pattern like working imu app)
// =============================================================================

#[test]
#[ignore]
fn test_accelerometer() {
    init_logger();

    let mut imu = BNO08x::new_spi_from_symbol(TEST_SPI_DEVICE, TEST_INT_GPIO, TEST_RST_GPIO)
        .expect("Failed to create IMU driver");
    imu.init().expect("Failed to initialize IMU");

    imu.enable_report(SENSOR_REPORTID_ACCELEROMETER, REPORT_INTERVAL_MS)
        .expect("Failed to enable accelerometer");

    sleep(Duration::from_millis(SENSOR_WARMUP_MS));

    // Read messages using pattern from working imu application
    for _ in 0..10 {
        imu.handle_one_message(REPORT_INTERVAL_MS as usize * 2);
        sleep(Duration::from_millis(REPORT_INTERVAL_MS as u64));
    }

    let accel = imu.accelerometer().expect("No accelerometer data");
    let magnitude = (accel[0].powi(2) + accel[1].powi(2) + accel[2].powi(2)).sqrt();

    assert!(
        magnitude > 8.0 && magnitude < 12.0,
        "Accelerometer magnitude {} outside expected range",
        magnitude
    );

    println!("✓ Accelerometer: {:?}, |a| = {:.2} m/s²", accel, magnitude);
}

#[test]
#[ignore]
fn test_gyroscope() {
    init_logger();

    let mut imu = BNO08x::new_spi_from_symbol(TEST_SPI_DEVICE, TEST_INT_GPIO, TEST_RST_GPIO)
        .expect("Failed to create IMU driver");
    imu.init().expect("Failed to initialize IMU");

    imu.enable_gyro(REPORT_INTERVAL_MS)
        .expect("Failed to enable gyroscope");
    sleep(Duration::from_millis(SENSOR_WARMUP_MS));

    for _ in 0..10 {
        imu.handle_one_message(REPORT_INTERVAL_MS as usize * 2);
        sleep(Duration::from_millis(REPORT_INTERVAL_MS as u64));
    }

    let gyro = imu.gyro().expect("No gyroscope data");

    // At rest, angular velocity should be small
    assert!(
        gyro[0].abs() < 1.0 && gyro[1].abs() < 1.0 && gyro[2].abs() < 1.0,
        "Gyroscope readings {:?} too high for stationary sensor",
        gyro
    );

    println!("✓ Gyroscope: {:?} rad/s", gyro);
}

#[test]
#[ignore]
fn test_magnetometer() {
    init_logger();

    let mut imu = BNO08x::new_spi_from_symbol(TEST_SPI_DEVICE, TEST_INT_GPIO, TEST_RST_GPIO)
        .expect("Failed to create IMU driver");
    imu.init().expect("Failed to initialize IMU");

    imu.enable_report(SENSOR_REPORTID_MAGNETIC_FIELD, REPORT_INTERVAL_MS)
        .expect("Failed to enable magnetometer");

    sleep(Duration::from_millis(SENSOR_WARMUP_MS));

    for _ in 0..10 {
        imu.handle_one_message(REPORT_INTERVAL_MS as usize * 2);
        sleep(Duration::from_millis(REPORT_INTERVAL_MS as u64));
    }

    let mag = imu.mag_field().expect("No magnetometer data");
    let magnitude = (mag[0].powi(2) + mag[1].powi(2) + mag[2].powi(2)).sqrt();

    // Earth's magnetic field is typically 25-65 µT
    assert!(
        magnitude > 10.0 && magnitude < 100.0,
        "Magnetic field {} outside expected range",
        magnitude
    );

    println!("✓ Magnetometer: {:?}, |B| = {:.2} µT", mag, magnitude);
}

#[test]
#[ignore]
fn test_rotation_vector() {
    init_logger();

    let mut imu = BNO08x::new_spi_from_symbol(TEST_SPI_DEVICE, TEST_INT_GPIO, TEST_RST_GPIO)
        .expect("Failed to create IMU driver");
    imu.init().expect("Failed to initialize IMU");

    imu.enable_rotation_vector(REPORT_INTERVAL_MS)
        .expect("Failed to enable rotation vector");

    sleep(Duration::from_millis(SENSOR_WARMUP_MS * 2));

    for _ in 0..10 {
        imu.handle_one_message(REPORT_INTERVAL_MS as usize * 2);
        sleep(Duration::from_millis(REPORT_INTERVAL_MS as u64));
    }

    let quat = imu.rotation_quaternion().expect("No rotation quaternion");
    let acc = imu.rotation_acc();

    let magnitude = (quat[0].powi(2) + quat[1].powi(2) + quat[2].powi(2) + quat[3].powi(2)).sqrt();
    assert!(
        (magnitude - 1.0).abs() < 0.1,
        "Quaternion magnitude {} should be close to 1.0",
        magnitude
    );

    println!(
        "✓ Rotation: {:?}, |q| = {:.3}, acc = {:.3} rad",
        quat, magnitude, acc
    );
}

#[test]
#[ignore]
fn test_linear_acceleration_and_gravity() {
    init_logger();

    let mut imu = BNO08x::new_spi_from_symbol(TEST_SPI_DEVICE, TEST_INT_GPIO, TEST_RST_GPIO)
        .expect("Failed to create IMU driver");
    imu.init().expect("Failed to initialize IMU");

    imu.enable_linear_accel(REPORT_INTERVAL_MS)
        .expect("Failed to enable linear accel");
    imu.enable_gravity(REPORT_INTERVAL_MS)
        .expect("Failed to enable gravity");

    sleep(Duration::from_millis(SENSOR_WARMUP_MS));

    for _ in 0..10 {
        imu.handle_one_message(REPORT_INTERVAL_MS as usize * 2);
        sleep(Duration::from_millis(REPORT_INTERVAL_MS as u64));
    }

    let linear = imu.linear_accel().expect("No linear accel data");
    let gravity = imu.gravity().expect("No gravity data");

    let linear_mag = (linear[0].powi(2) + linear[1].powi(2) + linear[2].powi(2)).sqrt();
    let gravity_mag = (gravity[0].powi(2) + gravity[1].powi(2) + gravity[2].powi(2)).sqrt();

    // At rest, linear acceleration should be near zero
    assert!(
        linear_mag < 2.0,
        "Linear accel too high for stationary sensor"
    );

    // Gravity should be ~9.8 m/s²
    assert!(
        gravity_mag > 8.0 && gravity_mag < 11.0,
        "Gravity magnitude {} outside expected range",
        gravity_mag
    );

    println!(
        "✓ Linear accel: {:?}, |a_lin| = {:.2} m/s²",
        linear, linear_mag
    );
    println!("✓ Gravity: {:?}, |g| = {:.2} m/s²", gravity, gravity_mag);
}

// =============================================================================
// API Usage Tests (matching working imu application patterns)
// =============================================================================

#[test]
#[ignore]
fn test_handle_messages_with_limit() {
    init_logger();

    let mut imu = BNO08x::new_spi_from_symbol(TEST_SPI_DEVICE, TEST_INT_GPIO, TEST_RST_GPIO)
        .expect("Failed to create IMU driver");
    imu.init().expect("Failed to initialize IMU");

    imu.enable_report(SENSOR_REPORTID_ACCELEROMETER, REPORT_INTERVAL_MS)
        .expect("Failed to enable accelerometer");

    sleep(Duration::from_millis(SENSOR_WARMUP_MS));

    // This is how the working imu application does it: handle_messages(timeout,
    // max_count)
    let count = imu.handle_messages(REPORT_INTERVAL_MS as usize * 2, 5);

    assert!(
        count > 0 && count <= 5,
        "Should have handled 1-5 messages, got {}",
        count
    );
    println!("✓ handle_messages(timeout, max=5): {} messages", count);
}

#[test]
#[ignore]
fn test_sensor_report_callback() {
    init_logger();

    let mut imu = BNO08x::new_spi_from_symbol(TEST_SPI_DEVICE, TEST_INT_GPIO, TEST_RST_GPIO)
        .expect("Failed to create IMU driver");
    imu.init().expect("Failed to initialize IMU");

    let callback_count = Arc::new(Mutex::new(0));
    let callback_count_clone = callback_count.clone();

    // This is how the working imu application adds callbacks
    imu.add_sensor_report_callback(
        SENSOR_REPORTID_ACCELEROMETER,
        "test_callback".to_string(),
        move |_imu| {
            let mut count = callback_count_clone.lock().unwrap();
            *count += 1;
        },
    );

    imu.enable_report(SENSOR_REPORTID_ACCELEROMETER, REPORT_INTERVAL_MS)
        .expect("Failed to enable accelerometer");

    sleep(Duration::from_millis(SENSOR_WARMUP_MS));

    // Read some data - callbacks should be triggered
    for _ in 0..10 {
        imu.handle_one_message(REPORT_INTERVAL_MS as usize * 2);
        sleep(Duration::from_millis(REPORT_INTERVAL_MS as u64));
    }

    let count = *callback_count.lock().unwrap();
    assert!(
        count > 0,
        "Callback should have been triggered, got {}",
        count
    );

    println!("✓ Callback triggered {} times", count);

    imu.remove_sensor_report_callback(SENSOR_REPORTID_ACCELEROMETER, "test_callback".to_string());
    println!("✓ Callback removed successfully");
}

#[test]
#[ignore]
fn test_report_status_queries() {
    init_logger();

    let mut imu = BNO08x::new_spi_from_symbol(TEST_SPI_DEVICE, TEST_INT_GPIO, TEST_RST_GPIO)
        .expect("Failed to create IMU driver");
    imu.init().expect("Failed to initialize IMU");

    // Initially disabled
    assert!(!imu.is_report_enabled(SENSOR_REPORTID_ACCELEROMETER));
    assert_eq!(imu.report_update_time(SENSOR_REPORTID_ACCELEROMETER), 0);

    // Enable
    imu.enable_report(SENSOR_REPORTID_ACCELEROMETER, REPORT_INTERVAL_MS)
        .expect("Failed to enable accelerometer");
    sleep(Duration::from_millis(100));

    assert!(imu.is_report_enabled(SENSOR_REPORTID_ACCELEROMETER));

    // Read data
    for _ in 0..5 {
        imu.handle_one_message(REPORT_INTERVAL_MS as usize * 2);
        sleep(Duration::from_millis(REPORT_INTERVAL_MS as u64));
    }

    let update_time = imu.report_update_time(SENSOR_REPORTID_ACCELEROMETER);
    assert!(
        update_time > 0,
        "Update time should be non-zero after receiving data"
    );

    println!("✓ is_report_enabled() and report_update_time() work correctly");
    println!("  Update time: {} ms", update_time);
}

#[test]
#[ignore]
fn test_multiple_sensors() {
    init_logger();

    let mut imu = BNO08x::new_spi_from_symbol(TEST_SPI_DEVICE, TEST_INT_GPIO, TEST_RST_GPIO)
        .expect("Failed to create IMU driver");
    imu.init().expect("Failed to initialize IMU");

    // Enable multiple sensors like the working imu application does
    imu.enable_report(SENSOR_REPORTID_ACCELEROMETER, REPORT_INTERVAL_MS)
        .expect("accel");
    imu.enable_gyro(REPORT_INTERVAL_MS).expect("gyro");
    imu.enable_rotation_vector(REPORT_INTERVAL_MS)
        .expect("rotation");

    sleep(Duration::from_millis(SENSOR_WARMUP_MS));

    // Use handle_messages like working app
    for _ in 0..20 {
        imu.handle_messages(REPORT_INTERVAL_MS as usize * 2, 10);
        sleep(Duration::from_millis(REPORT_INTERVAL_MS as u64));
    }

    // All sensors should have data
    let accel = imu.accelerometer().expect("No accel");
    let gyro = imu.gyro().expect("No gyro");
    let quat = imu.rotation_quaternion().expect("No rotation");

    println!("✓ Multiple sensors:");
    println!("  Accel: {:?}", accel);
    println!("  Gyro: {:?}", gyro);
    println!("  Rotation: {:?}", quat);
}
