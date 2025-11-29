// Copyright 2025 Au-Zone Technologies Inc.
// SPDX-License-Identifier: Apache-2.0

//! BNO08x IMU driver implementation.
//!
//! This module contains the main driver for the BNO08x family of IMU sensors.
//! It provides a high-level API for initializing the sensor, enabling reports,
//! and reading sensor data.

use crate::{
    constants::{
        frs_status_to_str, q_to_f32, CHANNEL_COMMAND, CHANNEL_EXECUTABLE, CHANNEL_HUB_CONTROL,
        CHANNEL_SENSOR_REPORTS, CMD_RESP_ADVERTISEMENT, CMD_RESP_ERROR_LIST,
        EXECUTABLE_DEVICE_CMD_RESET, EXECUTABLE_DEVICE_RESP_RESET_COMPLETE, FRS_STATUS_NO_DATA,
        FRS_STATUS_WRITE_COMPLETE, FRS_STATUS_WRITE_FAILED, FRS_STATUS_WRITE_READY, NUM_CHANNELS,
        PACKET_RECV_BUF_LEN, PACKET_SEND_BUF_LEN, Q_POINTS, Q_POINTS2,
        SENSOR_REPORTID_ACCELEROMETER, SENSOR_REPORTID_GRAVITY, SENSOR_REPORTID_GYROSCOPE,
        SENSOR_REPORTID_GYROSCOPE_UNCALIB, SENSOR_REPORTID_LINEAR_ACCEL,
        SENSOR_REPORTID_MAGNETIC_FIELD, SENSOR_REPORTID_ROTATION_VECTOR,
        SENSOR_REPORTID_ROTATION_VECTOR_GAME, SENSOR_REPORTID_ROTATION_VECTOR_GEOMAGNETIC,
        SH2_INIT_SYSTEM, SH2_STARTUP_INIT_UNSOLICITED, SHUB_COMMAND_RESP, SHUB_FRS_WRITE_RESP,
        SHUB_GET_FEATURE_RESP, SHUB_PROD_ID_REQ, SHUB_PROD_ID_RESP, SHUB_REPORT_SET_FEATURE_CMD,
    },
    frs::{
        build_frs_write_data, build_frs_write_request, quaternion_to_frs_words,
        FRS_TYPE_SENSOR_ORIENTATION,
    },
    interface::{
        delay::delay_ms,
        gpio::{GpiodIn, GpiodOut},
        spi::SpiControlLines,
        spidev::SpiDevice,
        SensorInterface, SpiInterface, PACKET_HEADER_LENGTH,
    },
};
use log::{debug, trace, warn};

use core::ops::Shr;
use std::{
    collections::HashMap,
    fmt::Debug,
    io::{self, Error, ErrorKind},
    time::{Instant, SystemTime},
};

/// Type alias for sensor update callback functions
type ReportCallbackMap<'a, SI> = HashMap<String, Box<dyn Fn(&BNO08x<'a, SI>) + 'a>>;

/// Driver-level errors
#[derive(Debug)]
pub enum DriverError<E> {
    /// Communications error
    CommError(E),
    /// Invalid chip ID was read
    InvalidChipId(u8),
    /// Unsupported sensor firmware version
    InvalidFWVersion(u8),
    /// We expected some data but didn't receive any
    NoDataAvailable,
}

/// BNO08x IMU driver
///
/// This struct provides the main interface for communicating with BNO08x
/// family IMU sensors over SPI or I2C.
pub struct BNO08x<'a, SI> {
    pub(crate) sensor_interface: SI,
    /// Each communication channel with the device has its own sequence number
    sequence_numbers: [u8; NUM_CHANNELS],
    /// Buffer for building and sending packets to the sensor hub
    packet_send_buf: [u8; PACKET_SEND_BUF_LEN],
    /// Buffer for building packets received from the sensor hub
    packet_recv_buf: [u8; PACKET_RECV_BUF_LEN],

    last_packet_len_received: usize,
    /// Has the device been successfully reset
    device_reset: bool,
    /// Has the product ID been verified
    prod_id_verified: bool,

    init_received: bool,

    /// Have we received the full advertisement
    advert_received: bool,

    /// Is the device ready to do an FRS write
    frs_write_status: u8,

    /// Have we received an error list
    error_list_received: bool,
    last_error_received: u8,

    last_chan_received: u8,
    last_exec_chan_rid: u8,
    last_command_chan_rid: u8,

    /// Accelerometer data [x, y, z] in m/s^2
    accelerometer: [f32; 3],

    /// Rotation vector as unit quaternion [i, j, k, real]
    rotation_quaternion: [f32; 4],
    /// Rotation vector accuracy estimate (radians)
    rotation_acc: f32,

    /// Geomagnetic rotation vector as unit quaternion [i, j, k, real]
    geomag_rotation_quaternion: [f32; 4],
    /// Geomagnetic rotation accuracy estimate (radians)
    geomag_rotation_acc: f32,

    /// Game rotation vector as unit quaternion [i, j, k, real]
    game_rotation_quaternion: [f32; 4],

    /// Linear acceleration [x, y, z] in m/s^2 (gravity removed)
    linear_accel: [f32; 3],

    /// Gravity vector [x, y, z] in m/s^2
    gravity: [f32; 3],

    /// Calibrated gyroscope data [x, y, z] in rad/s
    gyro: [f32; 3],

    /// Uncalibrated gyroscope data [x, y, z] in rad/s
    uncalib_gyro: [f32; 3],

    /// Calibrated magnetic field [x, y, z] in uTesla
    mag_field: [f32; 3],

    /// Which reports are enabled
    report_enabled: [bool; 16],

    /// Timestamp of last update for each report
    report_update_time: [u128; 16],

    /// Callbacks for report updates
    report_update_callbacks: [ReportCallbackMap<'a, SI>; 16],
}

impl<SI> BNO08x<'_, SI> {
    /// Create a new BNO08x driver with the given sensor interface
    pub fn new_with_interface(sensor_interface: SI) -> Self {
        Self {
            sensor_interface,
            sequence_numbers: [0; NUM_CHANNELS],
            packet_send_buf: [0; PACKET_SEND_BUF_LEN],
            packet_recv_buf: [0; PACKET_RECV_BUF_LEN],
            last_packet_len_received: 0,
            device_reset: false,
            prod_id_verified: false,
            frs_write_status: FRS_STATUS_NO_DATA,
            init_received: false,
            advert_received: false,
            error_list_received: false,
            last_error_received: 0,
            last_chan_received: 0,
            last_exec_chan_rid: 0,
            last_command_chan_rid: 0,
            accelerometer: [0.0; 3],
            rotation_quaternion: [0.0; 4],
            rotation_acc: 0.0,
            game_rotation_quaternion: [0.0; 4],
            geomag_rotation_quaternion: [0.0; 4],
            geomag_rotation_acc: 0.0,
            linear_accel: [0.0; 3],
            gravity: [0.0; 3],
            gyro: [0.0; 3],
            uncalib_gyro: [0.0; 3],
            mag_field: [0.0; 3],
            report_enabled: [false; 16],
            report_update_time: [0; 16],
            report_update_callbacks: std::array::from_fn(|_| HashMap::new()),
        }
    }

    /// Returns previously consumed sensor interface instance.
    pub fn free(self) -> SI {
        self.sensor_interface
    }
}

impl<'a> BNO08x<'a, SpiInterface<SpiDevice, GpiodIn, GpiodOut>> {
    /// Create a new BNO08x driver using SPI with explicit GPIO chip and pin
    /// numbers
    ///
    /// # Arguments
    /// * `spidevice` - Path to the SPI device (e.g., "/dev/spidev1.0")
    /// * `hintn_gpiochip` - GPIO chip for the interrupt pin
    /// * `hintn_pin` - GPIO pin number for the interrupt
    /// * `reset_gpiochip` - GPIO chip for the reset pin
    /// * `reset_pin` - GPIO pin number for reset
    pub fn new_spi(
        spidevice: &str,
        hintn_gpiochip: &str,
        hintn_pin: u32,
        reset_gpiochip: &str,
        reset_pin: u32,
    ) -> io::Result<BNO08x<'a, SpiInterface<SpiDevice, GpiodIn, GpiodOut>>> {
        let hintn: GpiodIn;
        let reset: GpiodOut;
        if hintn_gpiochip == reset_gpiochip {
            let chip = gpiod::Chip::new(hintn_gpiochip)?;
            hintn = GpiodIn::new(&chip, hintn_pin)?;
            reset = GpiodOut::new(&chip, reset_pin)?;
        } else {
            let chip0 = gpiod::Chip::new(hintn_gpiochip)?;
            hintn = GpiodIn::new(&chip0, hintn_pin)?;
            let chip1 = gpiod::Chip::new(reset_gpiochip)?;
            reset = GpiodOut::new(&chip1, reset_pin)?;
        }

        let spidev = SpiDevice::new(spidevice)?;
        let ctrl_lines: SpiControlLines<SpiDevice, GpiodIn, GpiodOut> =
            SpiControlLines::<SpiDevice, GpiodIn, GpiodOut> {
                spi: spidev,
                hintn,
                reset,
            };

        let spi_int: SpiInterface<SpiDevice, GpiodIn, GpiodOut> = SpiInterface::new(ctrl_lines);
        let imu_driver: BNO08x<SpiInterface<SpiDevice, GpiodIn, GpiodOut>> =
            BNO08x::new_with_interface(spi_int);

        Ok(imu_driver)
    }

    /// Create a new BNO08x driver using SPI with GPIO pin names (symbol lookup)
    ///
    /// This method searches for GPIO pins by their symbolic names across all
    /// GPIO chips on the system.
    ///
    /// # Arguments
    /// * `spidevice` - Path to the SPI device (e.g., "/dev/spidev1.0")
    /// * `hintn_pin` - Symbolic name of the interrupt pin (e.g., "IMU_INT")
    /// * `reset_pin` - Symbolic name of the reset pin (e.g., "IMU_RST")
    pub fn new_spi_from_symbol(
        spidevice: &str,
        hintn_pin: &str,
        reset_pin: &str,
    ) -> io::Result<BNO08x<'a, SpiInterface<SpiDevice, GpiodIn, GpiodOut>>> {
        let gpio_chips = gpiod::Chip::list_devices()?;
        let mut hintn_gpio_chip = String::from("");
        let mut hintn_num = 0;
        let mut hintn_found = false;
        let mut reset_gpio_chip = String::from("");
        let mut reset_num = 0;
        let mut reset_found = false;
        'outer: for entry in gpio_chips {
            let chip = gpiod::Chip::new(&entry)?;
            for i in 0..chip.num_lines() {
                if !hintn_found && chip.line_info(i)?.name == hintn_pin {
                    hintn_gpio_chip = entry.display().to_string();
                    hintn_num = i;
                    hintn_found = true;
                } else if !reset_found && chip.line_info(i)?.name == reset_pin {
                    reset_gpio_chip = entry.display().to_string();
                    reset_num = i;
                    reset_found = true;
                }
                trace!("--- {} ---", chip.line_info(i)?.name);
                if reset_found && hintn_found {
                    break 'outer;
                }
            }
        }
        if !hintn_found {
            return Err(Error::new(
                ErrorKind::AddrNotAvailable,
                format!("Did not find hintn pin \"{}\"", hintn_pin),
            ));
        }
        if !reset_found {
            return Err(Error::new(
                ErrorKind::AddrNotAvailable,
                format!("Did not find reset pin \"{}\"", reset_pin),
            ));
        }
        Self::new_spi(
            spidevice,
            hintn_gpio_chip.as_str(),
            hintn_num,
            reset_gpio_chip.as_str(),
            reset_num,
        )
    }
}

impl<'a, SI, SE> BNO08x<'a, SI>
where
    SI: SensorInterface<SensorError = SE>,
    SE: core::fmt::Debug,
{
    /// Consume all available messages on the port without processing them
    pub fn eat_all_messages(&mut self) {
        loop {
            let msg_count = self.eat_one_message();
            if msg_count == 0 {
                break;
            }
            delay_ms(1);
        }
    }

    /// Handle up to `max_count` messages with the given timeout
    pub fn handle_messages(&mut self, timeout_ms: usize, max_count: u32) -> u32 {
        let mut total_handled: u32 = 0;
        let mut i: u32 = 0;
        while i < max_count {
            let handled_count = self.handle_one_message(timeout_ms);
            if handled_count == 0 || total_handled > max_count {
                break;
            } else {
                total_handled += handled_count;
                delay_ms(1);
            }
            i += 1
        }
        total_handled
    }

    /// Handle any messages with a timeout
    pub fn handle_all_messages(&mut self, timeout_ms: usize) -> u32 {
        let mut total_handled: u32 = 0;
        loop {
            let handled_count = self.handle_one_message(timeout_ms);
            if handled_count == 0 {
                break;
            } else {
                total_handled += handled_count;
                delay_ms(1);
            }
        }
        total_handled
    }

    /// Handle one message and return the count of messages handled (0 or 1)
    pub fn handle_one_message(&mut self, max_ms: usize) -> u32 {
        let mut msg_count = 0;

        let res = self.receive_packet_with_timeout(max_ms);
        if let Ok(received_len) = res {
            if received_len > 0 {
                msg_count += 1;
                if let Err(e) = self.handle_received_packet(received_len) {
                    warn!("{:?}", e)
                }
            }
        } else {
            trace!("handle1 err {:?}", res);
        }

        msg_count
    }

    /// Receive and ignore one message, returning the packet size or zero
    pub fn eat_one_message(&mut self) -> usize {
        let res = self.receive_packet_with_timeout(150);
        if let Ok(received_len) = res {
            received_len
        } else {
            trace!("e1 err {:?}", res);
            0
        }
    }

    fn handle_advertise_response(&mut self, received_len: usize) {
        let payload_len = received_len - PACKET_HEADER_LENGTH;
        let payload = &self.packet_recv_buf[PACKET_HEADER_LENGTH..received_len];
        let mut cursor: usize = 1; // skip response type

        while cursor < payload_len {
            let _tag: u8 = payload[cursor];
            cursor += 1;
            let len: u8 = payload[cursor];
            cursor += 1;
            cursor += len as usize;
        }

        self.advert_received = true;
    }

    fn read_u8_at_cursor(msg: &[u8], cursor: &mut usize) -> u8 {
        let val = msg[*cursor];
        *cursor += 1;
        val
    }

    fn read_i16_at_cursor(msg: &[u8], cursor: &mut usize) -> i16 {
        let val = (msg[*cursor] as i16) | ((msg[*cursor + 1] as i16) << 8);
        *cursor += 2;
        val
    }

    fn try_read_i16_at_cursor(msg: &[u8], cursor: &mut usize) -> Option<i16> {
        let remaining = msg.len() - *cursor;
        if remaining >= 2 {
            let val = (msg[*cursor] as i16) | ((msg[*cursor + 1] as i16) << 8);
            *cursor += 2;
            Some(val)
        } else {
            None
        }
    }

    /// Read data values from a single input report
    fn handle_one_input_report(
        outer_cursor: usize,
        msg: &[u8],
    ) -> (usize, u8, i16, i16, i16, i16, i16) {
        let mut cursor = outer_cursor;

        let feature_report_id = Self::read_u8_at_cursor(msg, &mut cursor);
        let _rep_seq_num = Self::read_u8_at_cursor(msg, &mut cursor);
        let _rep_status = Self::read_u8_at_cursor(msg, &mut cursor);
        let _delay = Self::read_u8_at_cursor(msg, &mut cursor);

        let data1: i16 = Self::read_i16_at_cursor(msg, &mut cursor);
        let data2: i16 = Self::read_i16_at_cursor(msg, &mut cursor);
        let data3: i16 = Self::read_i16_at_cursor(msg, &mut cursor);
        let data4: i16 = Self::try_read_i16_at_cursor(msg, &mut cursor).unwrap_or(0);
        let data5: i16 = Self::try_read_i16_at_cursor(msg, &mut cursor).unwrap_or(0);

        (cursor, feature_report_id, data1, data2, data3, data4, data5)
    }

    fn handle_sensor_report_update(&mut self, report_id: u8, timestamp: u128) {
        self.report_update_time[report_id as usize] = timestamp;
        for (_, val) in self.report_update_callbacks[report_id as usize].iter() {
            val(self);
        }
    }

    /// Handle parsing of an input report packet (may contain multiple reports)
    fn handle_sensor_reports(&mut self, received_len: usize) {
        let mut outer_cursor: usize = PACKET_HEADER_LENGTH + 5; // skip header, timestamp
        if received_len < outer_cursor {
            return;
        }

        let payload_len = received_len - outer_cursor;
        if payload_len < 10 {
            trace!(
                "bad report: {:?}",
                &self.packet_recv_buf[..PACKET_HEADER_LENGTH]
            );
            return;
        }

        while outer_cursor < payload_len {
            let (inner_cursor, report_id, data1, data2, data3, data4, data5) =
                Self::handle_one_input_report(outer_cursor, &self.packet_recv_buf[..received_len]);
            outer_cursor = inner_cursor;

            let timestamp = SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .map(|d| d.as_nanos())
                .unwrap_or(0);

            match report_id {
                SENSOR_REPORTID_ACCELEROMETER => {
                    self.update_accelerometer(data1, data2, data3);
                    self.handle_sensor_report_update(report_id, timestamp)
                }
                SENSOR_REPORTID_ROTATION_VECTOR => {
                    self.update_rotation_quaternion(data1, data2, data3, data4, data5);
                    self.handle_sensor_report_update(report_id, timestamp)
                }
                SENSOR_REPORTID_ROTATION_VECTOR_GAME => {
                    self.update_rotation_quaternion_game(data1, data2, data3, data4);
                    self.handle_sensor_report_update(report_id, timestamp)
                }
                SENSOR_REPORTID_ROTATION_VECTOR_GEOMAGNETIC => {
                    self.update_rotation_quaternion_geomag(data1, data2, data3, data4, data5);
                    self.handle_sensor_report_update(report_id, timestamp)
                }
                SENSOR_REPORTID_LINEAR_ACCEL => {
                    self.update_linear_accel(data1, data2, data3);
                    self.handle_sensor_report_update(report_id, timestamp)
                }
                SENSOR_REPORTID_GRAVITY => {
                    self.update_gravity(data1, data2, data3);
                    self.handle_sensor_report_update(report_id, timestamp)
                }
                SENSOR_REPORTID_GYROSCOPE => {
                    self.update_gyro_calib(data1, data2, data3);
                    self.handle_sensor_report_update(report_id, timestamp)
                }
                SENSOR_REPORTID_GYROSCOPE_UNCALIB => {
                    self.update_gyro_uncalib(data1, data2, data3);
                    self.handle_sensor_report_update(report_id, timestamp)
                }
                SENSOR_REPORTID_MAGNETIC_FIELD => {
                    self.update_magnetic_field_calib(data1, data2, data3);
                    self.handle_sensor_report_update(report_id, timestamp)
                }
                _ => {}
            }
        }
    }

    fn update_accelerometer(&mut self, x: i16, y: i16, z: i16) {
        let q = Q_POINTS[SENSOR_REPORTID_ACCELEROMETER as usize];
        self.accelerometer = [q_to_f32(x, q), q_to_f32(y, q), q_to_f32(z, q)];
    }

    fn update_rotation_quaternion(&mut self, q_i: i16, q_j: i16, q_k: i16, q_r: i16, q_a: i16) {
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

    fn update_rotation_quaternion_geomag(
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

    fn update_rotation_quaternion_game(&mut self, q_i: i16, q_j: i16, q_k: i16, q_r: i16) {
        let q = Q_POINTS[SENSOR_REPORTID_ROTATION_VECTOR_GAME as usize];
        self.game_rotation_quaternion = [
            q_to_f32(q_i, q),
            q_to_f32(q_j, q),
            q_to_f32(q_k, q),
            q_to_f32(q_r, q),
        ];
    }

    fn update_linear_accel(&mut self, x: i16, y: i16, z: i16) {
        let q = Q_POINTS[SENSOR_REPORTID_LINEAR_ACCEL as usize];
        self.linear_accel = [q_to_f32(x, q), q_to_f32(y, q), q_to_f32(z, q)];
    }

    fn update_gravity(&mut self, x: i16, y: i16, z: i16) {
        let q = Q_POINTS[SENSOR_REPORTID_GRAVITY as usize];
        self.gravity = [q_to_f32(x, q), q_to_f32(y, q), q_to_f32(z, q)];
    }

    fn update_gyro_calib(&mut self, x: i16, y: i16, z: i16) {
        let q = Q_POINTS[SENSOR_REPORTID_GYROSCOPE as usize];
        self.gyro = [q_to_f32(x, q), q_to_f32(y, q), q_to_f32(z, q)];
    }

    fn update_gyro_uncalib(&mut self, x: i16, y: i16, z: i16) {
        let q = Q_POINTS[SENSOR_REPORTID_GYROSCOPE_UNCALIB as usize];
        self.uncalib_gyro = [q_to_f32(x, q), q_to_f32(y, q), q_to_f32(z, q)];
    }

    fn update_magnetic_field_calib(&mut self, x: i16, y: i16, z: i16) {
        let q = Q_POINTS[SENSOR_REPORTID_MAGNETIC_FIELD as usize];
        self.mag_field = [q_to_f32(x, q), q_to_f32(y, q), q_to_f32(z, q)];
    }

    /// Handle one or more errors sent in response to a command
    fn handle_cmd_resp_error_list(&mut self, received_len: usize) {
        let payload_len = received_len - PACKET_HEADER_LENGTH;
        let payload = &self.packet_recv_buf[PACKET_HEADER_LENGTH..received_len];

        self.error_list_received = true;
        for err in payload.iter().take(payload_len).skip(1) {
            let err: u8 = *err;
            self.last_error_received = err;
            match err {
                0 => {}
                1 => {
                    warn!("Hub application attempted to exceed maximum read cargo length: Error code {}", err);
                }
                2 => {
                    warn!(
                        "Host write was too short (need at least a 4-byte header): Error code {}",
                        err
                    );
                }
                3 => {
                    warn!("Host wrote a header with length greater than maximum write cargo length: Error code {}", err);
                }
                4 => {
                    warn!("Host wrote a header with length less than or equal to header length: Error code {}", err);
                }
                5 => {
                    warn!("Host wrote beginning of fragmented cargo, fragmentation not supported: Error code {}", err);
                }
                6 => {
                    warn!("Host wrote continuation of fragmented cargo, fragmentation not supported: Error code {}", err);
                }
                7 => {
                    warn!(
                        "Unrecognized command on control channel: Error code {}",
                        err
                    );
                }
                8 => {
                    warn!(
                        "Unrecognized parameter to get-advertisement command: Error code {}",
                        err
                    );
                }
                9 => {
                    warn!("Host wrote to unrecognized channel: Error code {}", err);
                }
                10 => {
                    warn!("Advertisement request received while Advertisement Response was pending: Error code {}", err);
                }
                11 => {
                    warn!("Host performed a write operation before the hub had finished sending its advertisement response: Error code {}", err);
                }
                12 => {
                    warn!("Error list too long to send, truncated: Error code {}", err);
                }
                _ => {
                    debug!("Unknown error code {}", err);
                }
            }
        }
    }

    /// Handle a received packet and dispatch to appropriate handler
    pub fn handle_received_packet(&mut self, received_len: usize) -> Result<(), Box<dyn Debug>> {
        let mut rec_len = received_len;
        if rec_len > PACKET_RECV_BUF_LEN {
            warn!(
                "Packet length of {} exceeded the buffer length of {}",
                received_len, PACKET_RECV_BUF_LEN
            );
            rec_len = PACKET_RECV_BUF_LEN;
        } else if rec_len < PACKET_HEADER_LENGTH {
            return Err(Box::new(format!(
                "Packet length of {} was ignored. Shorter than header length of {}",
                received_len, PACKET_HEADER_LENGTH
            )));
        }
        let msg = &self.packet_recv_buf[..rec_len];
        let chan_num = msg[2];
        let report_id: u8 = if rec_len > PACKET_HEADER_LENGTH {
            msg[4]
        } else {
            0
        };
        self.last_chan_received = chan_num;
        match chan_num {
            CHANNEL_COMMAND => match report_id {
                CMD_RESP_ADVERTISEMENT => {
                    self.handle_advertise_response(rec_len);
                }
                CMD_RESP_ERROR_LIST => {
                    self.handle_cmd_resp_error_list(rec_len);
                }
                _ => {
                    self.last_command_chan_rid = report_id;
                    return Err(Box::new(format!("unknown cmd: {}", report_id)));
                }
            },
            CHANNEL_EXECUTABLE => match report_id {
                EXECUTABLE_DEVICE_RESP_RESET_COMPLETE => {
                    self.device_reset = true;
                    trace!("resp_reset {}", 1);
                }
                _ => {
                    self.last_exec_chan_rid = report_id;
                    return Err(Box::new(format!("unknown exe: {}", report_id)));
                }
            },
            CHANNEL_HUB_CONTROL => match report_id {
                SHUB_COMMAND_RESP => {
                    let cmd_resp = msg[6];
                    if cmd_resp == SH2_STARTUP_INIT_UNSOLICITED || cmd_resp == SH2_INIT_SYSTEM {
                        self.init_received = true;
                    }
                    trace!("CMD_RESP: 0x{:X}", cmd_resp);
                }
                SHUB_PROD_ID_RESP => {
                    {
                        let _sw_vers_major = msg[4 + 2];
                        let _sw_vers_minor = msg[4 + 3];
                        trace!("PID_RESP {}.{}", _sw_vers_major, _sw_vers_major);
                    }
                    self.prod_id_verified = true;
                }
                SHUB_GET_FEATURE_RESP => {
                    trace!("feat resp: {}", msg[5]);
                    self.report_enabled[msg[5] as usize] = true;
                }
                SHUB_FRS_WRITE_RESP => {
                    trace!("write resp: {}", frs_status_to_str(msg[5]));
                    self.frs_write_status = msg[5];
                }
                _ => {
                    trace!(
                        "unh hbc: 0x{:X} {:x?}",
                        report_id,
                        &msg[..PACKET_HEADER_LENGTH]
                    );
                    return Err(Box::new(format!(
                        "unknown hbc: 0x{:X} {:x?}",
                        report_id,
                        &msg[..PACKET_HEADER_LENGTH]
                    )));
                }
            },
            CHANNEL_SENSOR_REPORTS => {
                self.handle_sensor_reports(rec_len);
            }
            _ => {
                self.last_chan_received = chan_num;
                trace!("unh chan 0x{:X}", chan_num);
                return Err(Box::new(format!("unknown chan 0x{:X}", chan_num)));
            }
        }
        Ok(())
    }

    /// Initialize the BNO08x sensor.
    ///
    /// The BNO080 starts up with all sensors disabled, waiting for the
    /// application to configure it.
    pub fn init(&mut self) -> Result<(), DriverError<SE>> {
        trace!("driver init");

        // Section 5.1.1.1: On system startup, the SHTP control application will send
        // its full advertisement response, unsolicited, to the host.
        delay_ms(1);
        self.sensor_interface
            .setup()
            .map_err(DriverError::CommError)?;

        if self.sensor_interface.requires_soft_reset() {
            delay_ms(1);
            self.soft_reset()?;
            delay_ms(250);
            self.eat_all_messages();
            delay_ms(250);
            self.eat_all_messages();
        } else {
            // we only expect two messages after reset:
            // eat the advertisement response
            delay_ms(250);
            trace!("Eating advertisement response");
            self.handle_one_message(20);
            trace!("Eating reset response");
            delay_ms(250);
            self.handle_one_message(20);
        }
        self.verify_product_id()?;
        delay_ms(100);
        Ok(())
    }

    /// Enable reporting of rotation vector (fused quaternion).
    ///
    /// Note that the maximum valid update rate is 1 kHz, based on the max
    /// update rate of the sensor's gyros.
    ///
    /// Returns true if the report was successfully enabled.
    pub fn enable_rotation_vector(
        &mut self,
        millis_between_reports: u16,
    ) -> Result<bool, DriverError<SE>> {
        self.enable_report(SENSOR_REPORTID_ROTATION_VECTOR, millis_between_reports)
    }

    /// Enable reporting of linear acceleration vector.
    ///
    /// Returns true if the report was successfully enabled.
    pub fn enable_linear_accel(
        &mut self,
        millis_between_reports: u16,
    ) -> Result<bool, DriverError<SE>> {
        self.enable_report(SENSOR_REPORTID_LINEAR_ACCEL, millis_between_reports)
    }

    /// Enable reporting of calibrated gyroscope data.
    ///
    /// Returns true if the report was successfully enabled.
    pub fn enable_gyro(&mut self, millis_between_reports: u16) -> Result<bool, DriverError<SE>> {
        self.enable_report(SENSOR_REPORTID_GYROSCOPE, millis_between_reports)
    }

    /// Enable reporting of gravity vector.
    ///
    /// Returns true if the report was successfully enabled.
    pub fn enable_gravity(&mut self, millis_between_reports: u16) -> Result<bool, DriverError<SE>> {
        self.enable_report(SENSOR_REPORTID_GRAVITY, millis_between_reports)
    }

    /// Get the timestamp of the last update for a report
    pub fn report_update_time(&self, report_id: u8) -> u128 {
        if report_id as usize <= self.report_enabled.len() {
            return self.report_update_time[report_id as usize];
        }
        0
    }

    /// Check if a report is enabled
    pub fn is_report_enabled(&self, report_id: u8) -> bool {
        if report_id as usize <= self.report_enabled.len() {
            return self.report_enabled[report_id as usize];
        }
        false
    }

    /// Add a callback to be invoked when a sensor report is updated
    pub fn add_sensor_report_callback(
        &mut self,
        report_id: u8,
        key: String,
        func: impl Fn(&Self) + 'a,
    ) {
        self.report_update_callbacks[report_id as usize]
            .entry(key)
            .or_insert_with(|| Box::new(func));
    }

    /// Remove a sensor report callback by key
    pub fn remove_sensor_report_callback(&mut self, report_id: u8, key: String) {
        self.report_update_callbacks[report_id as usize].remove(&key);
    }

    /// Enable a sensor report with the specified update interval.
    ///
    /// Returns true if the report was successfully enabled.
    pub fn enable_report(
        &mut self,
        report_id: u8,
        millis_between_reports: u16,
    ) -> Result<bool, DriverError<SE>> {
        trace!("enable_report 0x{:X}", report_id);

        let micros_between_reports: u32 = (millis_between_reports as u32) * 1000;
        let cmd_body: [u8; 17] = [
            SHUB_REPORT_SET_FEATURE_CMD,
            report_id,
            0,                                        // feature flags
            0,                                        // LSB change sensitivity
            0,                                        // MSB change sensitivity
            (micros_between_reports & 0xFFu32) as u8, // LSB report interval, microseconds
            (micros_between_reports.shr(8) & 0xFFu32) as u8,
            (micros_between_reports.shr(16) & 0xFFu32) as u8,
            (micros_between_reports.shr(24) & 0xFFu32) as u8, // MSB report interval
            0,                                                // LSB Batch Interval
            0,
            0,
            0, // MSB Batch interval
            0, // LSB sensor-specific config
            0,
            0,
            0, // MSB sensor-specific config
        ];
        self.send_packet(CHANNEL_HUB_CONTROL, &cmd_body)?;

        let start = Instant::now();
        while !self.report_enabled[report_id as usize] && start.elapsed().as_millis() < 2000 {
            if let Ok(received_len) = self.receive_packet_with_timeout(250) {
                if received_len > 0 {
                    if let Err(e) = self.handle_received_packet(received_len) {
                        warn!("{:?}", e)
                    }
                }
            }
        }
        delay_ms(200);
        trace!(
            "Report {:x} is enabled: {}",
            report_id,
            self.report_enabled[report_id as usize]
        );
        if !self.report_enabled[report_id as usize] {
            return Ok(false);
        }
        Ok(true)
    }

    /// Set the sensor orientation using a quaternion.
    ///
    /// This configures the reference frame transformation applied to all
    /// sensor outputs.
    pub fn set_sensor_orientation(
        &mut self,
        qi: f32,
        qj: f32,
        qk: f32,
        qr: f32,
        timeout: u128,
    ) -> Result<bool, DriverError<SE>> {
        let length: u16 = 4;
        let cmd_body_req = build_frs_write_request(length, FRS_TYPE_SENSOR_ORIENTATION);
        let _ = self.send_packet(CHANNEL_HUB_CONTROL, cmd_body_req.as_ref())?;

        self.frs_write_status = FRS_STATUS_NO_DATA;
        let mut start = Instant::now();
        while self.frs_write_status == FRS_STATUS_NO_DATA && start.elapsed().as_millis() < timeout {
            if let Ok(received_len) = self.receive_packet_with_timeout(250) {
                if received_len > 0 {
                    if let Err(e) = self.handle_received_packet(received_len) {
                        warn!("{:?}", e)
                    }
                }
            }
        }

        if self.frs_write_status != FRS_STATUS_WRITE_READY {
            trace!("FRS Write not ready");
            return Ok(false);
        }
        trace!("FRS Write ready");
        delay_ms(150);

        let (q30_qi, q30_qj, q30_qk, q30_qr) = quaternion_to_frs_words(qi, qj, qk, qr);

        let mut offset: u16 = 0;
        let cmd_body_data = build_frs_write_data(offset, q30_qi, q30_qj);
        _ = self.send_packet(CHANNEL_HUB_CONTROL, cmd_body_data.as_ref())?;

        self.frs_write_status = FRS_STATUS_NO_DATA;
        start = Instant::now();
        while self.frs_write_status == FRS_STATUS_NO_DATA && start.elapsed().as_millis() < 800 {
            if let Ok(received_len) = self.receive_packet_with_timeout(250) {
                if received_len > 0 {
                    if let Err(e) = self.handle_received_packet(received_len) {
                        warn!("{:?}", e)
                    }
                }
            }
        }
        delay_ms(150);

        offset += 2;
        let cmd_body_data = build_frs_write_data(offset, q30_qk, q30_qr);
        _ = self.send_packet(CHANNEL_HUB_CONTROL, cmd_body_data.as_ref())?;

        self.frs_write_status = FRS_STATUS_NO_DATA;
        start = Instant::now();
        while self.frs_write_status != FRS_STATUS_WRITE_FAILED
            && self.frs_write_status != FRS_STATUS_WRITE_COMPLETE
            && start.elapsed().as_millis() < 800
        {
            if let Ok(received_len) = self.receive_packet_with_timeout(250) {
                if received_len > 0 {
                    if let Err(e) = self.handle_received_packet(received_len) {
                        warn!("{:?}", e)
                    }
                }
            }
        }
        delay_ms(100);
        Ok(self.frs_write_status == FRS_STATUS_WRITE_COMPLETE)
    }

    /// Prepare a packet for sending, in our send buffer
    fn prep_send_packet(&mut self, channel: u8, body_data: &[u8]) -> usize {
        let body_len = body_data.len();

        let packet_length = body_len + PACKET_HEADER_LENGTH;
        let packet_header = [
            (packet_length & 0xFF) as u8, // LSB
            packet_length.shr(8) as u8,   // MSB
            channel,
            self.sequence_numbers[channel as usize],
        ];
        self.sequence_numbers[channel as usize] += 1;

        self.packet_send_buf[..PACKET_HEADER_LENGTH].copy_from_slice(packet_header.as_ref());
        self.packet_send_buf[PACKET_HEADER_LENGTH..packet_length].copy_from_slice(body_data);

        packet_length
    }

    /// Send packet from our packet send buf
    fn send_packet(&mut self, channel: u8, body_data: &[u8]) -> Result<usize, DriverError<SE>> {
        let packet_length = self.prep_send_packet(channel, body_data);

        let rc = self
            .sensor_interface
            .send_and_receive_packet(
                &self.packet_send_buf[..packet_length],
                &mut self.packet_recv_buf,
            )
            .map_err(DriverError::CommError)?;
        if rc > 0 {
            if let Err(e) = self.handle_received_packet(rc) {
                warn!("{:?}", e)
            }
        }
        Ok(packet_length)
    }

    /// Read one packet into the receive buffer
    pub(crate) fn receive_packet_with_timeout(
        &mut self,
        max_ms: usize,
    ) -> Result<usize, DriverError<SE>> {
        self.packet_recv_buf[0] = 0;
        self.packet_recv_buf[1] = 0;
        let packet_len = self
            .sensor_interface
            .read_with_timeout(&mut self.packet_recv_buf, max_ms)
            .map_err(DriverError::CommError)?;

        self.last_packet_len_received = packet_len;

        Ok(packet_len)
    }

    /// Verify that the sensor returns an expected chip ID
    fn verify_product_id(&mut self) -> Result<(), DriverError<SE>> {
        trace!("request PID...");
        let cmd_body: [u8; 2] = [
            SHUB_PROD_ID_REQ, // request product ID
            0,                // reserved
        ];

        // for some reason, reading PID right after sending request does not work with
        // i2c
        if self.sensor_interface.requires_soft_reset() {
            self.send_packet(CHANNEL_HUB_CONTROL, cmd_body.as_ref())?;
        } else {
            let response_size =
                self.send_and_receive_packet(CHANNEL_HUB_CONTROL, cmd_body.as_ref())?;
            if response_size > 0 {
                if let Err(e) = self.handle_received_packet(response_size) {
                    warn!("{:?}", e)
                }
            }
        };

        // process all incoming messages until we get a product id (or no more data)
        while !self.prod_id_verified {
            trace!("read PID");
            let msg_count = self.handle_one_message(150);
            if msg_count < 1 {
                break;
            }
        }

        if !self.prod_id_verified {
            return Err(DriverError::InvalidChipId(0));
        }
        Ok(())
    }

    /// Get accelerometer data [x, y, z] in m/s^2
    pub fn accelerometer(&self) -> Result<[f32; 3], DriverError<SE>> {
        Ok(self.accelerometer)
    }

    /// Get rotation quaternion [i, j, k, real] (unit quaternion)
    pub fn rotation_quaternion(&self) -> Result<[f32; 4], DriverError<SE>> {
        Ok(self.rotation_quaternion)
    }

    /// Get rotation accuracy estimate in radians
    pub fn rotation_acc(&self) -> f32 {
        self.rotation_acc
    }

    /// Get game rotation quaternion [i, j, k, real] (unit quaternion)
    pub fn game_rotation_quaternion(&self) -> Result<[f32; 4], DriverError<SE>> {
        Ok(self.game_rotation_quaternion)
    }

    /// Get geomagnetic rotation quaternion [i, j, k, real] (unit quaternion)
    pub fn geomag_rotation_quaternion(&self) -> Result<[f32; 4], DriverError<SE>> {
        Ok(self.geomag_rotation_quaternion)
    }

    /// Get geomagnetic rotation accuracy estimate in radians
    pub fn geomag_rotation_acc(&self) -> f32 {
        self.geomag_rotation_acc
    }

    /// Get linear acceleration [x, y, z] in m/s^2 (gravity removed)
    pub fn linear_accel(&self) -> Result<[f32; 3], DriverError<SE>> {
        Ok(self.linear_accel)
    }

    /// Get gravity vector [x, y, z] in m/s^2
    pub fn gravity(&self) -> Result<[f32; 3], DriverError<SE>> {
        Ok(self.gravity)
    }

    /// Get calibrated gyroscope data [x, y, z] in rad/s
    pub fn gyro(&self) -> Result<[f32; 3], DriverError<SE>> {
        Ok(self.gyro)
    }

    /// Get uncalibrated gyroscope data [x, y, z] in rad/s
    pub fn gyro_uncalib(&self) -> Result<[f32; 3], DriverError<SE>> {
        Ok(self.uncalib_gyro)
    }

    /// Get calibrated magnetic field [x, y, z] in uT (micro-Tesla)
    pub fn mag_field(&self) -> Result<[f32; 3], DriverError<SE>> {
        Ok(self.mag_field)
    }

    /// Tell the sensor to reset.
    ///
    /// Normally applications should not need to call this directly,
    /// as it is called during `init`.
    pub fn soft_reset(&mut self) -> Result<(), DriverError<SE>> {
        trace!("soft_reset");
        let data: [u8; 1] = [EXECUTABLE_DEVICE_CMD_RESET];
        let received_len = self.send_and_receive_packet(CHANNEL_EXECUTABLE, data.as_ref())?;
        if received_len > 0 {
            if let Err(e) = self.handle_received_packet(received_len) {
                warn!("{:?}", e)
            }
        }
        Ok(())
    }

    /// Send a packet and receive the response
    fn send_and_receive_packet(
        &mut self,
        channel: u8,
        body_data: &[u8],
    ) -> Result<usize, DriverError<SE>> {
        let send_packet_length = self.prep_send_packet(channel, body_data);

        let recv_packet_length = self
            .sensor_interface
            .send_and_receive_packet(
                self.packet_send_buf[..send_packet_length].as_ref(),
                &mut self.packet_recv_buf,
            )
            .map_err(DriverError::CommError)?;

        Ok(recv_packet_length)
    }
}

#[cfg(test)]
mod tests {
    // Tests for driver functionality will be added as needed
}
