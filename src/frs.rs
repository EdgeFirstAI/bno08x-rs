// Copyright 2025 Au-Zone Technologies Inc.
// SPDX-License-Identifier: Apache-2.0

//! Flash Record System (FRS) operations for the BNO08x driver.
//!
//! This module contains functions for writing configuration data to the
//! sensor's flash memory using the FRS protocol.

use core::ops::Shr;

use crate::constants::{
    f32_to_q, FRS_STATUS_NO_DATA, FRS_STATUS_WRITE_COMPLETE, FRS_STATUS_WRITE_FAILED,
    FRS_STATUS_WRITE_READY, SHUB_FRS_WRITE_DATA_REQ, SHUB_FRS_WRITE_REQ,
};

/// FRS record type for sensor orientation
pub const FRS_TYPE_SENSOR_ORIENTATION: u16 = 0x2D3E;

/// Build an FRS write request command
pub fn build_frs_write_request(length: u16, frs_type: u16) -> [u8; 6] {
    [
        SHUB_FRS_WRITE_REQ,      // FRS write request
        0,                       // reserved
        (length & 0xFF) as u8,   // length LSB
        length.shr(8) as u8,     // length MSB
        (frs_type & 0xFF) as u8, // FRS Type LSB
        frs_type.shr(8) as u8,   // FRS Type MSB
    ]
}

/// Build an FRS write data command with two 32-bit words
pub fn build_frs_write_data(offset: u16, data0: [u8; 4], data1: [u8; 4]) -> [u8; 12] {
    [
        SHUB_FRS_WRITE_DATA_REQ, // FRS write data request
        0,                       // reserved
        (offset & 0xFF) as u8,   // offset LSB
        offset.shr(8) as u8,     // offset MSB
        data0[0],                // data0 LSB
        data0[1],
        data0[2],
        data0[3], // data0 MSB
        data1[0], // data1 LSB
        data1[1],
        data1[2],
        data1[3], // data1 MSB
    ]
}

/// Convert quaternion components to FRS data words (Q30 format)
pub fn quaternion_to_frs_words(
    qi: f32,
    qj: f32,
    qk: f32,
    qr: f32,
) -> ([u8; 4], [u8; 4], [u8; 4], [u8; 4]) {
    (
        f32_to_q(qi, 30),
        f32_to_q(qj, 30),
        f32_to_q(qk, 30),
        f32_to_q(qr, 30),
    )
}

/// Check if FRS write status indicates ready to write
pub fn is_write_ready(status: u8) -> bool {
    status == FRS_STATUS_WRITE_READY
}

/// Check if FRS write status indicates completion
pub fn is_write_complete(status: u8) -> bool {
    status == FRS_STATUS_WRITE_COMPLETE
}

/// Check if FRS write status indicates failure
pub fn is_write_failed(status: u8) -> bool {
    status == FRS_STATUS_WRITE_FAILED
}

/// Check if FRS write status is still pending (no data received yet)
pub fn is_no_data(status: u8) -> bool {
    status == FRS_STATUS_NO_DATA
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::{
        FRS_STATUS_BUSY, FRS_STATUS_RECORD_INVALID, FRS_STATUS_RECORD_VALID,
        FRS_STATUS_WORD_RECEIVED,
    };

    #[test]
    fn test_build_frs_write_request() {
        let req = build_frs_write_request(4, FRS_TYPE_SENSOR_ORIENTATION);

        assert_eq!(req[0], SHUB_FRS_WRITE_REQ);
        assert_eq!(req[1], 0); // reserved
        assert_eq!(req[2], 4); // length LSB
        assert_eq!(req[3], 0); // length MSB
        assert_eq!(req[4], 0x3E); // FRS type LSB (0x2D3E)
        assert_eq!(req[5], 0x2D); // FRS type MSB
    }

    #[test]
    fn test_build_frs_write_request_large_values() {
        let req = build_frs_write_request(0x1234, 0xABCD);

        assert_eq!(req[2], 0x34); // length LSB
        assert_eq!(req[3], 0x12); // length MSB
        assert_eq!(req[4], 0xCD); // FRS type LSB
        assert_eq!(req[5], 0xAB); // FRS type MSB
    }

    #[test]
    fn test_build_frs_write_data() {
        let data0 = [0x01, 0x02, 0x03, 0x04];
        let data1 = [0x05, 0x06, 0x07, 0x08];
        let cmd = build_frs_write_data(0, data0, data1);

        assert_eq!(cmd[0], SHUB_FRS_WRITE_DATA_REQ);
        assert_eq!(cmd[1], 0); // reserved
        assert_eq!(cmd[2], 0); // offset LSB
        assert_eq!(cmd[3], 0); // offset MSB
        assert_eq!(cmd[4..8], data0);
        assert_eq!(cmd[8..12], data1);
    }

    #[test]
    fn test_build_frs_write_data_with_offset() {
        let data0 = [0x11, 0x22, 0x33, 0x44];
        let data1 = [0x55, 0x66, 0x77, 0x88];
        let cmd = build_frs_write_data(0x0102, data0, data1);

        assert_eq!(cmd[2], 0x02); // offset LSB
        assert_eq!(cmd[3], 0x01); // offset MSB
    }

    #[test]
    fn test_quaternion_to_frs_words_identity() {
        // Identity quaternion: [0, 0, 0, 1]
        let (qi, qj, qk, qr) = quaternion_to_frs_words(0.0, 0.0, 0.0, 1.0);

        // qi, qj, qk should be 0
        assert_eq!(qi, [0, 0, 0, 0]);
        assert_eq!(qj, [0, 0, 0, 0]);
        assert_eq!(qk, [0, 0, 0, 0]);
        // qr should be 1.0 in Q30 format = 0x40000000
        assert_eq!(qr, [0x00, 0x00, 0x00, 0x40]);
    }

    #[test]
    fn test_quaternion_to_frs_words_half() {
        // Test with 0.5 values
        let (qi, _qj, _qk, _qr) = quaternion_to_frs_words(0.5, 0.5, 0.5, 0.5);

        // 0.5 in Q30 = 0x20000000
        assert_eq!(qi, [0x00, 0x00, 0x00, 0x20]);
    }

    #[test]
    fn test_is_write_ready() {
        assert!(is_write_ready(FRS_STATUS_WRITE_READY));
        assert!(!is_write_ready(FRS_STATUS_WRITE_COMPLETE));
        assert!(!is_write_ready(FRS_STATUS_WRITE_FAILED));
        assert!(!is_write_ready(FRS_STATUS_NO_DATA));
    }

    #[test]
    fn test_is_write_complete() {
        assert!(is_write_complete(FRS_STATUS_WRITE_COMPLETE));
        assert!(!is_write_complete(FRS_STATUS_WRITE_READY));
        assert!(!is_write_complete(FRS_STATUS_WRITE_FAILED));
        assert!(!is_write_complete(FRS_STATUS_NO_DATA));
    }

    #[test]
    fn test_is_write_failed() {
        assert!(is_write_failed(FRS_STATUS_WRITE_FAILED));
        assert!(!is_write_failed(FRS_STATUS_WRITE_COMPLETE));
        assert!(!is_write_failed(FRS_STATUS_WRITE_READY));
        assert!(!is_write_failed(FRS_STATUS_NO_DATA));
    }

    #[test]
    fn test_is_no_data() {
        assert!(is_no_data(FRS_STATUS_NO_DATA));
        assert!(!is_no_data(FRS_STATUS_WRITE_READY));
        assert!(!is_no_data(FRS_STATUS_WRITE_COMPLETE));
        assert!(!is_no_data(FRS_STATUS_WRITE_FAILED));
    }

    #[test]
    fn test_frs_status_coverage() {
        // Ensure the status check functions return false for other status values
        assert!(!is_write_ready(FRS_STATUS_WORD_RECEIVED));
        assert!(!is_write_ready(FRS_STATUS_BUSY));
        assert!(!is_write_ready(FRS_STATUS_RECORD_VALID));
        assert!(!is_write_ready(FRS_STATUS_RECORD_INVALID));
    }
}
