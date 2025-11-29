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
