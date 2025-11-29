// Copyright 2025 Au-Zone Technologies Inc.
// SPDX-License-Identifier: Apache-2.0

//! Sensor report handling for the BNO08x driver.
//!
//! This module contains structures and functions for parsing and storing
//! sensor data received from the BNO08x IMU.

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
