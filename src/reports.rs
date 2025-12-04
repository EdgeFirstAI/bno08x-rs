// Copyright 2025 Au-Zone Technologies Inc.
// SPDX-License-Identifier: Apache-2.0

//! Sensor report handling for the BNO08x driver.
//!
//! This module contains structures and functions for parsing and storing
//! sensor data received from the BNO08x IMU. The main types are:
//!
//! - [`SensorData`] - Storage for all sensor values (accelerometer, gyroscope,
//!   quaternions, etc.)
//! - [`ReportState`] - Tracks which reports are enabled and manages callbacks
//! - [`ReportParser`] - Helper functions for parsing binary report data
//!
//! # Data Formats
//!
//! The BNO08x reports sensor data in Q-point fixed-point format, where values
//! are scaled by 2^Q. This module handles the conversion to standard floating
//! point values:
//!
//! | Sensor | Q-Point | Range |
//! |--------|---------|-------|
//! | Accelerometer | Q8 | ±8g |
//! | Gyroscope | Q9 | ±2000°/s |
//! | Magnetometer | Q4 | ±2500µT |
//! | Rotation Vector | Q14 | Unit quaternion |
//!
//! # Example
//!
//! ```no_run
//! use bno08x_rs::SensorData;
//!
//! let data = SensorData::new();
//! // After reading from sensor:
//! // data.accelerometer contains [x, y, z] in m/s²
//! // data.rotation_quaternion contains [i, j, k, real] unit quaternion
//! ```

use std::collections::HashMap;

use crate::constants::{
    q_to_f32, Q_POINTS, Q_POINTS2, SENSOR_REPORTID_ACCELEROMETER, SENSOR_REPORTID_GRAVITY,
    SENSOR_REPORTID_GYROSCOPE, SENSOR_REPORTID_GYROSCOPE_UNCALIB, SENSOR_REPORTID_LINEAR_ACCEL,
    SENSOR_REPORTID_MAGNETIC_FIELD, SENSOR_REPORTID_ROTATION_VECTOR,
    SENSOR_REPORTID_ROTATION_VECTOR_GAME, SENSOR_REPORTID_ROTATION_VECTOR_GEOMAGNETIC,
};

/// Type alias for sensor update callback functions
pub type ReportCallback<'a, T> = Box<dyn Fn(&T) + 'a>;

/// Type alias for the callback map
pub type ReportCallbackMap<'a, T> = HashMap<String, ReportCallback<'a, T>>;

/// Sensor data storage for all supported BNO08x reports
#[derive(Debug, Default)]
pub struct SensorData {
    /// Accelerometer data [x, y, z] in m/s²
    pub accelerometer: [f32; 3],

    /// Rotation vector as unit quaternion [i, j, k, real]
    pub rotation_quaternion: [f32; 4],
    /// Rotation vector accuracy estimate (radians)
    pub rotation_acc: f32,

    /// Geomagnetic rotation vector as unit quaternion [i, j, k, real]
    pub geomag_rotation_quaternion: [f32; 4],
    /// Geomagnetic rotation accuracy estimate (radians)
    pub geomag_rotation_acc: f32,

    /// Game rotation vector as unit quaternion [i, j, k, real]
    pub game_rotation_quaternion: [f32; 4],

    /// Linear acceleration [x, y, z] in m/s² (gravity removed)
    pub linear_accel: [f32; 3],

    /// Gravity vector [x, y, z] in m/s²
    pub gravity: [f32; 3],

    /// Calibrated gyroscope data [x, y, z] in rad/s
    pub gyro: [f32; 3],

    /// Uncalibrated gyroscope data [x, y, z] in rad/s
    pub uncalib_gyro: [f32; 3],

    /// Calibrated magnetic field [x, y, z] in µTesla
    pub mag_field: [f32; 3],
}

impl SensorData {
    /// Create a new SensorData instance with default values
    pub fn new() -> Self {
        Self::default()
    }

    /// Update accelerometer data from Q-point values
    pub fn update_accelerometer(&mut self, x: i16, y: i16, z: i16) {
        let q = Q_POINTS[SENSOR_REPORTID_ACCELEROMETER as usize];
        self.accelerometer = [q_to_f32(x, q), q_to_f32(y, q), q_to_f32(z, q)];
    }

    /// Update rotation quaternion from Q-point values
    pub fn update_rotation_quaternion(&mut self, q_i: i16, q_j: i16, q_k: i16, q_r: i16, q_a: i16) {
        let q = Q_POINTS[SENSOR_REPORTID_ROTATION_VECTOR as usize];
        let q2 = Q_POINTS2[SENSOR_REPORTID_ROTATION_VECTOR as usize];
        self.rotation_quaternion = [
            q_to_f32(q_i, q),
            q_to_f32(q_j, q),
            q_to_f32(q_k, q),
            q_to_f32(q_r, q),
        ];
        self.rotation_acc = q_to_f32(q_a, q2);
    }

    /// Update geomagnetic rotation quaternion from Q-point values
    pub fn update_rotation_quaternion_geomag(
        &mut self,
        q_i: i16,
        q_j: i16,
        q_k: i16,
        q_r: i16,
        q_a: i16,
    ) {
        let q = Q_POINTS[SENSOR_REPORTID_ROTATION_VECTOR_GEOMAGNETIC as usize];
        let q2 = Q_POINTS2[SENSOR_REPORTID_ROTATION_VECTOR_GEOMAGNETIC as usize];
        self.geomag_rotation_quaternion = [
            q_to_f32(q_i, q),
            q_to_f32(q_j, q),
            q_to_f32(q_k, q),
            q_to_f32(q_r, q),
        ];
        self.geomag_rotation_acc = q_to_f32(q_a, q2);
    }

    /// Update game rotation quaternion from Q-point values
    pub fn update_rotation_quaternion_game(&mut self, q_i: i16, q_j: i16, q_k: i16, q_r: i16) {
        let q = Q_POINTS[SENSOR_REPORTID_ROTATION_VECTOR_GAME as usize];
        self.game_rotation_quaternion = [
            q_to_f32(q_i, q),
            q_to_f32(q_j, q),
            q_to_f32(q_k, q),
            q_to_f32(q_r, q),
        ];
    }

    /// Update linear acceleration from Q-point values
    pub fn update_linear_accel(&mut self, x: i16, y: i16, z: i16) {
        let q = Q_POINTS[SENSOR_REPORTID_LINEAR_ACCEL as usize];
        self.linear_accel = [q_to_f32(x, q), q_to_f32(y, q), q_to_f32(z, q)];
    }

    /// Update gravity vector from Q-point values
    pub fn update_gravity(&mut self, x: i16, y: i16, z: i16) {
        let q = Q_POINTS[SENSOR_REPORTID_GRAVITY as usize];
        self.gravity = [q_to_f32(x, q), q_to_f32(y, q), q_to_f32(z, q)];
    }

    /// Update calibrated gyroscope from Q-point values
    pub fn update_gyro_calib(&mut self, x: i16, y: i16, z: i16) {
        let q = Q_POINTS[SENSOR_REPORTID_GYROSCOPE as usize];
        self.gyro = [q_to_f32(x, q), q_to_f32(y, q), q_to_f32(z, q)];
    }

    /// Update uncalibrated gyroscope from Q-point values
    pub fn update_gyro_uncalib(&mut self, x: i16, y: i16, z: i16) {
        let q = Q_POINTS[SENSOR_REPORTID_GYROSCOPE_UNCALIB as usize];
        self.uncalib_gyro = [q_to_f32(x, q), q_to_f32(y, q), q_to_f32(z, q)];
    }

    /// Update calibrated magnetic field from Q-point values
    pub fn update_magnetic_field_calib(&mut self, x: i16, y: i16, z: i16) {
        let q = Q_POINTS[SENSOR_REPORTID_MAGNETIC_FIELD as usize];
        self.mag_field = [q_to_f32(x, q), q_to_f32(y, q), q_to_f32(z, q)];
    }
}

/// Report state tracking
pub struct ReportState<'a, T> {
    /// Which reports are enabled
    pub enabled: [bool; 16],
    /// Timestamp of last update for each report
    pub update_time: [u128; 16],
    /// Callbacks for report updates
    pub callbacks: [ReportCallbackMap<'a, T>; 16],
}

impl<'a, T> ReportState<'a, T> {
    /// Create a new ReportState instance
    pub fn new() -> Self {
        Self {
            enabled: [false; 16],
            update_time: [0; 16],
            callbacks: std::array::from_fn(|_| HashMap::new()),
        }
    }

    /// Check if a report is enabled
    pub fn is_enabled(&self, report_id: u8) -> bool {
        if (report_id as usize) < self.enabled.len() {
            self.enabled[report_id as usize]
        } else {
            false
        }
    }

    /// Get the last update time for a report
    pub fn last_update_time(&self, report_id: u8) -> u128 {
        if (report_id as usize) < self.update_time.len() {
            self.update_time[report_id as usize]
        } else {
            0
        }
    }

    /// Mark a report as enabled
    pub fn set_enabled(&mut self, report_id: u8, enabled: bool) {
        if (report_id as usize) < self.enabled.len() {
            self.enabled[report_id as usize] = enabled;
        }
    }

    /// Update the timestamp for a report
    pub fn set_update_time(&mut self, report_id: u8, timestamp: u128) {
        if (report_id as usize) < self.update_time.len() {
            self.update_time[report_id as usize] = timestamp;
        }
    }

    /// Add a callback for a report
    pub fn add_callback(&mut self, report_id: u8, key: String, callback: ReportCallback<'a, T>) {
        if (report_id as usize) < self.callbacks.len() {
            self.callbacks[report_id as usize]
                .entry(key)
                .or_insert(callback);
        }
    }

    /// Remove a callback for a report
    pub fn remove_callback(&mut self, report_id: u8, key: &str) {
        if (report_id as usize) < self.callbacks.len() {
            self.callbacks[report_id as usize].remove(key);
        }
    }
}

impl<'a, T> Default for ReportState<'a, T> {
    fn default() -> Self {
        Self::new()
    }
}

/// Helper functions for parsing report data from raw bytes
pub struct ReportParser;

impl ReportParser {
    /// Read a u8 at the cursor position and advance cursor
    #[inline]
    pub fn read_u8(msg: &[u8], cursor: &mut usize) -> u8 {
        let val = msg[*cursor];
        *cursor += 1;
        val
    }

    /// Read an i16 (little-endian) at the cursor position and advance cursor
    #[inline]
    pub fn read_i16(msg: &[u8], cursor: &mut usize) -> i16 {
        let val = (msg[*cursor] as i16) | ((msg[*cursor + 1] as i16) << 8);
        *cursor += 2;
        val
    }

    /// Try to read an i16 if enough bytes remain, returns None otherwise
    #[inline]
    pub fn try_read_i16(msg: &[u8], cursor: &mut usize) -> Option<i16> {
        if msg.len() - *cursor >= 2 {
            Some(Self::read_i16(msg, cursor))
        } else {
            None
        }
    }

    /// Parse a single input report from the message buffer
    /// Returns (new_cursor, report_id, data1, data2, data3, data4, data5)
    pub fn parse_input_report(cursor: usize, msg: &[u8]) -> (usize, u8, i16, i16, i16, i16, i16) {
        let mut pos = cursor;

        let report_id = Self::read_u8(msg, &mut pos);
        let _seq_num = Self::read_u8(msg, &mut pos);
        let _status = Self::read_u8(msg, &mut pos);
        let _delay = Self::read_u8(msg, &mut pos);

        let data1 = Self::read_i16(msg, &mut pos);
        let data2 = Self::read_i16(msg, &mut pos);
        let data3 = Self::read_i16(msg, &mut pos);
        let data4 = Self::try_read_i16(msg, &mut pos).unwrap_or(0);
        let data5 = Self::try_read_i16(msg, &mut pos).unwrap_or(0);

        (pos, report_id, data1, data2, data3, data4, data5)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sensor_data_new() {
        let data = SensorData::new();
        assert_eq!(data.accelerometer, [0.0, 0.0, 0.0]);
        assert_eq!(data.rotation_quaternion, [0.0, 0.0, 0.0, 0.0]);
        assert_eq!(data.gyro, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_update_accelerometer() {
        let mut data = SensorData::new();
        // Q8 format: 256 = 1.0
        data.update_accelerometer(256, 512, -256);
        assert!((data.accelerometer[0] - 1.0).abs() < 0.01);
        assert!((data.accelerometer[1] - 2.0).abs() < 0.01);
        assert!((data.accelerometer[2] + 1.0).abs() < 0.01);
    }

    #[test]
    fn test_update_rotation_quaternion() {
        let mut data = SensorData::new();
        // Q14 format: 16384 = 1.0
        data.update_rotation_quaternion(16384, 0, 0, 0, 0);
        assert!((data.rotation_quaternion[0] - 1.0).abs() < 0.001);
        assert!((data.rotation_quaternion[1]).abs() < 0.001);
        assert!((data.rotation_quaternion[2]).abs() < 0.001);
        assert!((data.rotation_quaternion[3]).abs() < 0.001);
    }

    #[test]
    fn test_update_gyro() {
        let mut data = SensorData::new();
        // Q9 format: 512 = 1.0 rad/s
        data.update_gyro_calib(512, -512, 256);
        assert!((data.gyro[0] - 1.0).abs() < 0.01);
        assert!((data.gyro[1] + 1.0).abs() < 0.01);
        assert!((data.gyro[2] - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_update_linear_accel() {
        let mut data = SensorData::new();
        data.update_linear_accel(256, 0, -256);
        assert!((data.linear_accel[0] - 1.0).abs() < 0.01);
        assert!((data.linear_accel[1]).abs() < 0.01);
        assert!((data.linear_accel[2] + 1.0).abs() < 0.01);
    }

    #[test]
    fn test_update_gravity() {
        let mut data = SensorData::new();
        // Gravity is typically ~9.8 m/s² in Z axis
        let g_z = (9.8 * 256.0) as i16; // Q8 format
        data.update_gravity(0, 0, g_z);
        assert!((data.gravity[0]).abs() < 0.01);
        assert!((data.gravity[1]).abs() < 0.01);
        assert!((data.gravity[2] - 9.8).abs() < 0.1);
    }

    #[test]
    fn test_update_mag_field() {
        let mut data = SensorData::new();
        // Q4 format: 16 = 1.0 µT
        data.update_magnetic_field_calib(160, 320, -160);
        assert!((data.mag_field[0] - 10.0).abs() < 0.1);
        assert!((data.mag_field[1] - 20.0).abs() < 0.1);
        assert!((data.mag_field[2] + 10.0).abs() < 0.1);
    }

    #[test]
    fn test_read_u8() {
        let data = [0x12, 0x34, 0x56, 0x78];
        let mut pos = 0;
        assert_eq!(ReportParser::read_u8(&data, &mut pos), 0x12);
        assert_eq!(pos, 1);
        assert_eq!(ReportParser::read_u8(&data, &mut pos), 0x34);
        assert_eq!(pos, 2);
    }

    #[test]
    fn test_read_i16() {
        // Little-endian: 0x34, 0x12 -> 0x1234 as i16 = 4660
        let data = [0x34, 0x12, 0xFF, 0xFF];
        let mut pos = 0;
        assert_eq!(ReportParser::read_i16(&data, &mut pos), 0x1234u16 as i16);
        assert_eq!(pos, 2);
        // 0xFF, 0xFF -> -1
        assert_eq!(ReportParser::read_i16(&data, &mut pos), -1);
        assert_eq!(pos, 4);
    }

    #[test]
    fn test_try_read_i16() {
        let data = [0x34, 0x12];
        let mut pos = 0;
        assert_eq!(
            ReportParser::try_read_i16(&data, &mut pos),
            Some(0x1234u16 as i16)
        );
        assert_eq!(pos, 2);

        // Not enough data
        assert_eq!(ReportParser::try_read_i16(&data, &mut pos), None);
        assert_eq!(pos, 2); // pos unchanged on failure
    }

    #[test]
    fn test_parse_input_report() {
        // Construct a minimal input report
        let msg = [
            0x01, // report_id
            0x02, // seq_num
            0x03, // status
            0x04, // delay
            0x00, 0x01, // data1 (0x0100 = 256)
            0x00, 0x02, // data2 (0x0200 = 512)
            0x00, 0x03, // data3 (0x0300 = 768)
            0x00, 0x04, // data4 (0x0400 = 1024)
            0x00, 0x05, // data5 (0x0500 = 1280)
        ];

        let (pos, report_id, data1, data2, data3, data4, data5) =
            ReportParser::parse_input_report(0, &msg);

        assert_eq!(report_id, 0x01);
        assert_eq!(data1, 256);
        assert_eq!(data2, 512);
        assert_eq!(data3, 768);
        assert_eq!(data4, 1024);
        assert_eq!(data5, 1280);
        assert_eq!(pos, msg.len());
    }

    #[test]
    fn test_parse_input_report_short() {
        // Report with only 3 data fields
        let msg = [
            0x01, // report_id
            0x02, // seq_num
            0x03, // status
            0x04, // delay
            0x00, 0x01, // data1
            0x00, 0x02, // data2
            0x00, 0x03, // data3
        ];

        let (pos, report_id, data1, data2, data3, data4, data5) =
            ReportParser::parse_input_report(0, &msg);

        assert_eq!(report_id, 0x01);
        assert_eq!(data1, 256);
        assert_eq!(data2, 512);
        assert_eq!(data3, 768);
        assert_eq!(data4, 0); // Default when not enough data
        assert_eq!(data5, 0); // Default when not enough data
        assert_eq!(pos, msg.len());
    }

    #[test]
    fn test_quaternion_normalization() {
        // Unit quaternion should have magnitude 1.0
        let q: [f32; 4] = [0.5, 0.5, 0.5, 0.5];
        let mag_sq: f32 = q[0].powi(2) + q[1].powi(2) + q[2].powi(2) + q[3].powi(2);
        assert!((mag_sq - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_update_rotation_quaternion_geomag() {
        let mut data = SensorData::new();
        // Q14 format: 16384 = 1.0
        data.update_rotation_quaternion_geomag(0, 0, 0, 16384, 4096);
        assert!((data.geomag_rotation_quaternion[0]).abs() < 0.001);
        assert!((data.geomag_rotation_quaternion[1]).abs() < 0.001);
        assert!((data.geomag_rotation_quaternion[2]).abs() < 0.001);
        assert!((data.geomag_rotation_quaternion[3] - 1.0).abs() < 0.001);
        // Accuracy in Q12 format: 4096 = 1.0 radian
        assert!((data.geomag_rotation_acc - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_update_rotation_quaternion_game() {
        let mut data = SensorData::new();
        // Q14 format: 16384 = 1.0
        data.update_rotation_quaternion_game(8192, 8192, 0, 0);
        assert!((data.game_rotation_quaternion[0] - 0.5).abs() < 0.001);
        assert!((data.game_rotation_quaternion[1] - 0.5).abs() < 0.001);
        assert!((data.game_rotation_quaternion[2]).abs() < 0.001);
        assert!((data.game_rotation_quaternion[3]).abs() < 0.001);
    }

    #[test]
    fn test_update_gyro_uncalib() {
        let mut data = SensorData::new();
        // Q9 format: 512 = 1.0 rad/s
        data.update_gyro_uncalib(512, -512, 0);
        assert!((data.uncalib_gyro[0] - 1.0).abs() < 0.01);
        assert!((data.uncalib_gyro[1] + 1.0).abs() < 0.01);
        assert!((data.uncalib_gyro[2]).abs() < 0.01);
    }

    #[test]
    fn test_report_state_new() {
        let state: ReportState<'_, ()> = ReportState::new();
        
        // All reports should be disabled initially
        for i in 0..16u8 {
            assert!(!state.is_enabled(i));
            assert_eq!(state.last_update_time(i), 0);
        }
    }

    #[test]
    fn test_report_state_enable_disable() {
        let mut state: ReportState<'_, ()> = ReportState::new();
        
        state.set_enabled(1, true);
        assert!(state.is_enabled(1));
        assert!(!state.is_enabled(0));
        
        state.set_enabled(1, false);
        assert!(!state.is_enabled(1));
    }

    #[test]
    fn test_report_state_update_time() {
        let mut state: ReportState<'_, ()> = ReportState::new();
        
        state.set_update_time(5, 12345678);
        assert_eq!(state.last_update_time(5), 12345678);
        assert_eq!(state.last_update_time(0), 0);
    }

    #[test]
    fn test_report_state_bounds_checking() {
        let mut state: ReportState<'_, ()> = ReportState::new();
        
        // Test with out-of-bounds report ID
        assert!(!state.is_enabled(255));
        assert_eq!(state.last_update_time(255), 0);
        
        // These should not panic
        state.set_enabled(255, true);
        state.set_update_time(255, 1000);
        
        // Values should still be default (no effect)
        assert!(!state.is_enabled(255));
    }

    #[test]
    fn test_report_state_callback_management() {
        let mut state: ReportState<'_, i32> = ReportState::new();
        
        // Add a callback
        let called = std::rc::Rc::new(std::cell::Cell::new(false));
        let called_clone = called.clone();
        state.add_callback(1, "test".to_string(), Box::new(move |_| {
            called_clone.set(true);
        }));
        
        // Check callback exists (via callbacks map length)
        assert!(!state.callbacks[1].is_empty());
        assert!(state.callbacks[2].is_empty());
        
        // Remove callback
        state.remove_callback(1, "test");
        assert!(state.callbacks[1].is_empty());
    }

    #[test]
    fn test_report_state_callback_bounds() {
        let mut state: ReportState<'_, ()> = ReportState::new();
        
        // Out of bounds should not panic
        state.add_callback(255, "test".to_string(), Box::new(|_| {}));
        
        // Remove on out-of-bounds should not panic
        state.remove_callback(255, "test");
    }
}
