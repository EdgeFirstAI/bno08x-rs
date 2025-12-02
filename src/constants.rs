// Copyright 2025 Au-Zone Technologies Inc.
// SPDX-License-Identifier: Apache-2.0

//! Constants for the BNO08x sensor driver.
//!
//! This module contains all protocol constants, report IDs, channel
//! definitions, and Q-point values used for communication with the BNO08x
//! sensor.

/// Buffer sizes
pub const PACKET_SEND_BUF_LEN: usize = 256;
pub const PACKET_RECV_BUF_LEN: usize = 2048;
pub const NUM_CHANNELS: usize = 6;

// =============================================================================
// SHTP Communication Channels
// =============================================================================

/// The BNO080 supports six communication channels
pub const CHANNEL_COMMAND: u8 = 0;
/// The SHTP command channel
pub const CHANNEL_EXECUTABLE: u8 = 1;
/// Executable channel
pub const CHANNEL_HUB_CONTROL: u8 = 2;
/// Sensor hub control channel
pub const CHANNEL_SENSOR_REPORTS: u8 = 3;
// Input sensor reports (non-wake, not gyroRV)
// const CHANNEL_WAKE_REPORTS: usize = 4; // wake input sensor reports
// const CHANNEL_GYRO_ROTATION: usize = 5; // gyro rotation vector (gyroRV)

// =============================================================================
// Command Channel Responses
// =============================================================================

/// Advertisement response
pub const CMD_RESP_ADVERTISEMENT: u8 = 0;
/// Error list response
pub const CMD_RESP_ERROR_LIST: u8 = 1;

// =============================================================================
// Sensor Hub (SHUB) Protocol Constants
// =============================================================================

/// Report ID for Product ID request
pub const SHUB_PROD_ID_REQ: u8 = 0xF9;
/// Report ID for Product ID response
pub const SHUB_PROD_ID_RESP: u8 = 0xF8;
/// Get feature response
pub const SHUB_GET_FEATURE_RESP: u8 = 0xFC;
/// Set feature command
pub const SHUB_REPORT_SET_FEATURE_CMD: u8 = 0xFD;
/// Command response
pub const SHUB_COMMAND_RESP: u8 = 0xF1;
/// FRS write request
pub const SHUB_FRS_WRITE_REQ: u8 = 0xF7;
/// FRS write data request
pub const SHUB_FRS_WRITE_DATA_REQ: u8 = 0xF6;
/// FRS write response
pub const SHUB_FRS_WRITE_RESP: u8 = 0xF5;

// =============================================================================
// Sensor Report IDs (from SH2 Reference Manual)
// =============================================================================

/// Accelerometer (m/s^2 including gravity): Q point 8
pub const SENSOR_REPORTID_ACCELEROMETER: u8 = 0x01;
/// Gyroscope calibrated (rad/s): Q point 9
pub const SENSOR_REPORTID_GYROSCOPE: u8 = 0x02;
/// Magnetic field calibrated (uTesla): Q point 4
pub const SENSOR_REPORTID_MAGNETIC_FIELD: u8 = 0x03;
/// Linear acceleration (m/s^2 minus gravity): Q point 8
pub const SENSOR_REPORTID_LINEAR_ACCEL: u8 = 0x04;
/// Unit quaternion rotation vector, Q point 14, with heading accuracy (radians)
/// Q point 12
pub const SENSOR_REPORTID_ROTATION_VECTOR: u8 = 0x05;
/// Gravity vector: Q point 8
pub const SENSOR_REPORTID_GRAVITY: u8 = 0x06;
/// Gyroscope uncalibrated (rad/s): Q point 9
pub const SENSOR_REPORTID_GYROSCOPE_UNCALIB: u8 = 0x07;
/// Game rotation vector: Q point 14
pub const SENSOR_REPORTID_ROTATION_VECTOR_GAME: u8 = 0x08;
/// Geomagnetic rotation vector: Q point 14 for quaternion, Q point 12 for
/// heading accuracy
pub const SENSOR_REPORTID_ROTATION_VECTOR_GEOMAGNETIC: u8 = 0x09;

// =============================================================================
// FRS (Flash Record System) Status Codes
// =============================================================================

/// Word(s) received
pub const FRS_STATUS_WORD_RECEIVED: u8 = 0;
/// Unrecognized FRS type
pub const FRS_STATUS_UNRECOGNIZED_FRS_TYPE: u8 = 1;
/// Busy
pub const FRS_STATUS_BUSY: u8 = 2;
/// Write completed
pub const FRS_STATUS_WRITE_COMPLETE: u8 = 3;
/// Write mode entered or ready
pub const FRS_STATUS_WRITE_READY: u8 = 4;
/// Write failed
pub const FRS_STATUS_WRITE_FAILED: u8 = 5;
/// Data received while not in write mode
pub const FRS_STATUS_DATA_RECV_NOT_IN_WRITE_MODE: u8 = 6;
/// Invalid length
pub const FRS_STATUS_INVALID_LENGTH: u8 = 7;
/// Record valid (passed internal validation)
pub const FRS_STATUS_RECORD_VALID: u8 = 8;
/// Record invalid (failed internal validation)
pub const FRS_STATUS_RECORD_INVALID: u8 = 9;
/// Device error (DFU flash memory device unavailable)
pub const FRS_STATUS_DEVICE_ERROR: u8 = 10;
/// Record is read only
pub const FRS_STATUS_READONLY: u8 = 11;
/// Reserved
pub const FRS_STATUS_RESERVED: u8 = 12;
/// No FRS status received (internal sentinel)
pub const FRS_STATUS_NO_DATA: u8 = u8::MAX;

// =============================================================================
// Q-Point Tables for Fixed-Point Conversion
// =============================================================================

/// Q-points for primary sensor data (indexed by report ID)
pub const Q_POINTS: [usize; 15] = [0, 8, 9, 4, 8, 14, 8, 9, 14, 14, 0, 0, 0, 0, 0];
/// Q-points for secondary sensor data like accuracy (indexed by report ID)
pub const Q_POINTS2: [usize; 15] = [0, 0, 0, 0, 0, 12, 0, 0, 0, 12, 0, 0, 0, 0, 0];

// =============================================================================
// Executable/Device Channel Commands
// =============================================================================

/// Reset command
pub const EXECUTABLE_DEVICE_CMD_RESET: u8 = 1;
/// Reset complete response
pub const EXECUTABLE_DEVICE_RESP_RESET_COMPLETE: u8 = 1;

// =============================================================================
// Initialization Commands
// =============================================================================

/// Unsolicited flag
pub const SH2_INIT_UNSOLICITED: u8 = 0x80;
/// Initialize command
pub const SH2_CMD_INITIALIZE: u8 = 4;
/// System initialization
pub const SH2_INIT_SYSTEM: u8 = 1;
/// Startup initialization (unsolicited)
pub const SH2_STARTUP_INIT_UNSOLICITED: u8 = SH2_CMD_INITIALIZE | SH2_INIT_UNSOLICITED;

// =============================================================================
// Helper Functions
// =============================================================================

/// Convert Q-point fixed-point value to f32
#[inline]
pub fn q_to_f32(q_val: i16, q_point: usize) -> f32 {
    use std::ops::Shl;
    (q_val as f32) / (1.shl(q_point) as f32)
}

/// Convert f32 to Q-point fixed-point bytes (little-endian)
#[inline]
pub fn f32_to_q(f32_val: f32, q_point: usize) -> [u8; 4] {
    use std::ops::Shl;
    ((f32_val as f64 * (1.shl(q_point) as f64)) as i32).to_le_bytes()
}

/// Get FRS status description string
pub fn frs_status_to_str(status: u8) -> &'static str {
    match status {
        FRS_STATUS_WORD_RECEIVED => "word(s) received",
        FRS_STATUS_UNRECOGNIZED_FRS_TYPE => "unrecognized FRS type",
        FRS_STATUS_BUSY => "busy",
        FRS_STATUS_WRITE_COMPLETE => "write completed",
        FRS_STATUS_WRITE_READY => "write mode entered or ready",
        FRS_STATUS_WRITE_FAILED => "write failed",
        FRS_STATUS_DATA_RECV_NOT_IN_WRITE_MODE => "data received while not in write mode",
        FRS_STATUS_INVALID_LENGTH => "invalid length",
        FRS_STATUS_RECORD_VALID => "record valid (passed internal validation)",
        FRS_STATUS_RECORD_INVALID => "record invalid (failed internal validation)",
        FRS_STATUS_DEVICE_ERROR => "device error (DFU flash unavailable)",
        FRS_STATUS_READONLY => "record is read only",
        FRS_STATUS_NO_DATA => "no FRS status received",
        _ => "reserved",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_f32_to_q() {
        // f32_to_q returns little-endian bytes: 0.25 * 2^30 = 0x10000000
        // In little-endian: [0x00, 0x00, 0x00, 0x10]
        assert_eq!(
            [0x00, 0x00, 0x00, 0x10],
            f32_to_q(0.25, 30),
            "Wrong positive q point value"
        );
        // -0.25 * 2^30 = -0x10000000 = 0xF0000000 in two's complement
        // In little-endian: [0x00, 0x00, 0x00, 0xF0]
        assert_eq!(
            [0x00, 0x00, 0x00, 0xf0],
            f32_to_q(-0.25, 30),
            "Wrong negative q point value"
        );
        // Test zero
        assert_eq!([0x00, 0x00, 0x00, 0x00], f32_to_q(0.0, 30));
        // Test 1.0
        assert_eq!([0x00, 0x00, 0x00, 0x40], f32_to_q(1.0, 30));
    }

    #[test]
    fn test_q_to_f32() {
        // Q8: 256 in Q8 = 1.0
        assert!((q_to_f32(256, 8) - 1.0).abs() < 0.001);
        // Q14: 16384 in Q14 = 1.0
        assert!((q_to_f32(16384, 14) - 1.0).abs() < 0.001);
        // Test zero
        assert!((q_to_f32(0, 14)).abs() < 0.001);
        // Test negative values
        assert!((q_to_f32(-16384, 14) + 1.0).abs() < 0.001);
        // Test fractional
        assert!((q_to_f32(8192, 14) - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_q_point_roundtrip() {
        // Test that converting f32 -> Q -> f32 preserves value
        let test_values = [0.0, 0.5, 1.0, -0.5, -1.0, 0.123, -0.456];
        for &val in &test_values {
            let q_bytes = f32_to_q(val, 14);
            let q_val = i16::from_le_bytes([q_bytes[0], q_bytes[1]]);
            let result = q_to_f32(q_val, 14);
            assert!(
                (result - val).abs() < 0.001,
                "Roundtrip failed for {}: got {}",
                val,
                result
            );
        }
    }

    #[test]
    fn test_channel_constants() {
        // Verify channel IDs are unique
        assert_ne!(CHANNEL_COMMAND, CHANNEL_EXECUTABLE);
        assert_ne!(CHANNEL_COMMAND, CHANNEL_HUB_CONTROL);
        assert_ne!(CHANNEL_COMMAND, CHANNEL_SENSOR_REPORTS);
        assert_ne!(CHANNEL_EXECUTABLE, CHANNEL_HUB_CONTROL);
        assert_ne!(CHANNEL_EXECUTABLE, CHANNEL_SENSOR_REPORTS);
        assert_ne!(CHANNEL_HUB_CONTROL, CHANNEL_SENSOR_REPORTS);
        
        // Verify channels are in valid range
        assert!(CHANNEL_COMMAND < NUM_CHANNELS as u8);
        assert!(CHANNEL_EXECUTABLE < NUM_CHANNELS as u8);
        assert!(CHANNEL_HUB_CONTROL < NUM_CHANNELS as u8);
        assert!(CHANNEL_SENSOR_REPORTS < NUM_CHANNELS as u8);
    }

    #[test]
    fn test_sensor_report_ids() {
        // Verify report IDs are unique
        let report_ids = [
            SENSOR_REPORTID_ACCELEROMETER,
            SENSOR_REPORTID_GYROSCOPE,
            SENSOR_REPORTID_GYROSCOPE_UNCALIB,
            SENSOR_REPORTID_MAGNETIC_FIELD,
            SENSOR_REPORTID_LINEAR_ACCEL,
            SENSOR_REPORTID_ROTATION_VECTOR,
            SENSOR_REPORTID_ROTATION_VECTOR_GAME,
            SENSOR_REPORTID_ROTATION_VECTOR_GEOMAGNETIC,
            SENSOR_REPORTID_GRAVITY,
        ];
        
        for (i, &id1) in report_ids.iter().enumerate() {
            for &id2 in report_ids.iter().skip(i + 1) {
                assert_ne!(id1, id2, "Duplicate report ID found: {}", id1);
            }
        }
    }

    #[test]
    fn test_q_points_arrays() {
        // Verify Q_POINTS and Q_POINTS2 arrays exist and have entries
        assert!(Q_POINTS.len() > 0, "Q_POINTS should have entries");
        assert!(Q_POINTS2.len() > 0, "Q_POINTS2 should have entries");
        assert_eq!(Q_POINTS.len(), Q_POINTS2.len(), "Q_POINTS arrays should have same length");
        
        // Verify known report IDs have valid Q points
        assert!(Q_POINTS[SENSOR_REPORTID_ACCELEROMETER as usize] > 0);
        assert!(Q_POINTS[SENSOR_REPORTID_GYROSCOPE as usize] > 0);
        assert!(Q_POINTS[SENSOR_REPORTID_ROTATION_VECTOR as usize] > 0);
    }

    #[test]
    fn test_buffer_sizes() {
        // Verify buffer sizes are reasonable
        assert!(PACKET_SEND_BUF_LEN > 0);
        assert!(PACKET_RECV_BUF_LEN > 0);
        assert!(PACKET_RECV_BUF_LEN >= PACKET_SEND_BUF_LEN);
        
        // Receive buffer should be larger for handling sensor reports
        assert!(PACKET_RECV_BUF_LEN > PACKET_SEND_BUF_LEN);
    }
}
