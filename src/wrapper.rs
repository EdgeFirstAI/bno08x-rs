/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

/*
Modified 2023 Au-Zone Technologies
*/
use crate::interface::delay::delay_ms;
use crate::interface::gpio::{GpiodIn, GpiodOut};
use crate::interface::spi::SpiControlLines;
use crate::interface::spidev::SpiDevice;
use crate::interface::{SensorInterface, SpiInterface, PACKET_HEADER_LENGTH};
use crate::log;
use crate::wrapper::io::Error;

use core::ops::Shr;
use std::collections::HashMap;
use std::io::{self, ErrorKind};
use std::ops::Shl;
use std::time::{Instant, SystemTime};

const PACKET_SEND_BUF_LEN: usize = 256;
const PACKET_RECV_BUF_LEN: usize = 2048;

const NUM_CHANNELS: usize = 6;

#[derive(Debug)]
pub enum WrapperError<E> {
    ///Communications error
    CommError(E),
    /// Invalid chip ID was read
    InvalidChipId(u8),
    /// Unsupported sensor firmware version
    InvalidFWVersion(u8),
    /// We expected some data but didn't receive any
    NoDataAvailable,
}

pub struct BNO08x<'a, SI> {
    pub(crate) sensor_interface: SI,
    /// each communication channel with the device has its own sequence number
    sequence_numbers: [u8; NUM_CHANNELS],
    /// buffer for building and sending packet to the sensor hub
    packet_send_buf: [u8; PACKET_SEND_BUF_LEN],
    /// buffer for building packets received from the sensor hub
    packet_recv_buf: [u8; PACKET_RECV_BUF_LEN],

    last_packet_len_received: usize,
    /// has the device been succesfully reset
    device_reset: bool,
    /// has the product ID been verified
    prod_id_verified: bool,

    init_received: bool,

    /// have we received the full advertisement
    advert_received: bool,

    /// is the device ready to do an FRS write
    frs_write_status: u8,

    /// have we received an error list
    error_list_received: bool,
    last_error_received: u8,

    last_chan_received: u8,
    last_exec_chan_rid: u8,
    last_command_chan_rid: u8,

    /// Accelerometer
    accelerometer: [f32; 3],

    /// Rotation vector as unit quaternion
    rotation_quaternion: [f32; 4],
    rotation_acc: f32,

    /// Geomagnetic rotation vector
    geomag_rotation_quaternion: [f32; 4],
    geomag_rotation_acc: f32,

    /// Game rotation vector
    game_rotation_quaternion: [f32; 4],

    /// Linear acceleration vector
    linear_accel: [f32; 3],

    /// Gravity vector
    gravity: [f32; 3],

    /// Gyroscope calibrated data
    gyro: [f32; 3],

    uncalib_gryo: [f32; 3],

    /// Magnetic field calibrated data
    mag_field: [f32; 3],

    report_enabled: [bool; 16],

    /// When the last update for a report was, compare using
    report_update_time: [u128; 16],

    /// Sensor update callbacks
    report_update_callbacks:
        [HashMap<String, Box<dyn Fn(&Self) + 'a>>; 16],
}

impl<'a, SI> BNO08x<'a, SI> {
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
            uncalib_gryo: [0.0; 3],
            mag_field: [0.0; 3],
            report_enabled: [false; 16],
            report_update_time: [0; 16],
            report_update_callbacks: std::array::from_fn(|_| HashMap::new()),
        }
    }

    /// Returns previously consumed serial sensor instance.
    pub fn free(self) -> SI {
        self.sensor_interface
    }
}

impl<'a> BNO08x<'a, SpiInterface<SpiDevice, GpiodIn, GpiodOut>> {
    pub fn new_bno08x(
        spidevice: &str,
        hintn_gpiochip: &str,
        hintn_pin: u32,
        reset_gpiochip: &str,
        reset_pin: u32,
    ) -> io::Result<BNO08x<'a, SpiInterface<SpiDevice, GpiodIn, GpiodOut>>>
    {
        let hintn: GpiodIn;
        let reset: GpiodOut;
        if hintn_gpiochip == reset_gpiochip {
            let mut _chip = gpiod::Chip::new(hintn_gpiochip)?;
            hintn = GpiodIn::new(&_chip, hintn_pin)?;
            reset = GpiodOut::new(&_chip, reset_pin)?;
        } else {
            let mut _chip0 = gpiod::Chip::new(hintn_gpiochip)?;
            hintn = GpiodIn::new(&_chip0, hintn_pin)?;
            let mut _chip1 = gpiod::Chip::new(reset_gpiochip)?;
            reset = GpiodOut::new(&_chip1, reset_pin)?;
        }

        let mut _spidev = SpiDevice::new(spidevice)?;
        let ctrl_lines: SpiControlLines<SpiDevice, GpiodIn, GpiodOut> =
            SpiControlLines::<SpiDevice, GpiodIn, GpiodOut> {
                spi: _spidev,
                hintn,
                reset,
            };

        let spi_int: SpiInterface<SpiDevice, GpiodIn, GpiodOut> =
            SpiInterface::new(ctrl_lines);
        let imu_driver: BNO08x<SpiInterface<SpiDevice, GpiodIn, GpiodOut>> =
            BNO08x::new_with_interface(spi_int);

        Ok(imu_driver)
    }

    pub fn new_bno08x_from_symbol(
        spidevice: &str,
        hintn_pin: &str,
        reset_pin: &str,
    ) -> io::Result<BNO08x<'a, SpiInterface<SpiDevice, GpiodIn, GpiodOut>>>
    {
        let gpio_chips = gpiod::Chip::list_devices()?;
        let mut hintn_gpio_chip = String::from("");
        let mut hintn_num = 0;
        let mut hintn_found = false;
        let mut reset_gpio_chip = String::from("");
        let mut reset_num = 0;
        let mut reset_found = false;
        'outer: for entry in gpio_chips {
            let mut _chip = gpiod::Chip::new(&entry)?;
            for i in 0.._chip.num_lines() {
                if !hintn_found && _chip.line_info(i)?.name == hintn_pin {
                    hintn_gpio_chip = entry.display().to_string();
                    hintn_num = i;
                    hintn_found = true;
                } else if !reset_found && _chip.line_info(i)?.name == reset_pin
                {
                    reset_gpio_chip = entry.display().to_string();
                    reset_num = i;
                    reset_found = true;
                }
                log!("--- {} ---", _chip.line_info(i)?.name);
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
        Self::new_bno08x(
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
            //give some time to other parts of the system
            delay_ms(1);
        }
    }

    pub fn handle_messages(
        &mut self,
        timeout_ms: usize,
        max_count: u32,
    ) -> u32 {
        let mut total_handled: u32 = 0;
        let mut i: u32 = 0;
        while i < max_count {
            let handled_count = self.handle_one_message(timeout_ms);
            if handled_count == 0 || total_handled > max_count {
                break;
            } else {
                total_handled += handled_count;
                //give some time to other parts of the system
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
                //give some time to other parts of the system
                delay_ms(1);
            }
        }
        total_handled
    }

    /// return the number of messages handled
    pub fn handle_one_message(&mut self, max_ms: usize) -> u32 {
        let mut msg_count = 0;

        let res = self.receive_packet_with_timeout(max_ms);
        if res.is_ok() {
            let received_len = res.unwrap_or(0);
            if received_len > 0 {
                msg_count += 1;
                self.handle_received_packet(received_len);
            }
        } else {
            log!("handle1 err {:?}", res);
        }

        msg_count
    }

    /// Receive and ignore one message,
    /// returning the size of the packet received or zero
    /// if there was no packet to read.
    pub fn eat_one_message(&mut self) -> usize {
        let res = self.receive_packet_with_timeout(150);
        if let Ok(received_len) = res {
            received_len
        } else {
            log!("e1 err {:?}", res);
            0
        }
    }

    fn handle_advertise_response(&mut self, received_len: usize) {
        let payload_len = received_len - PACKET_HEADER_LENGTH;
        let payload = &self.packet_recv_buf[PACKET_HEADER_LENGTH..received_len];
        let mut cursor: usize = 1; //skip response type

        while cursor < payload_len {
            let _tag: u8 = payload[cursor];
            cursor += 1;
            let len: u8 = payload[cursor];
            cursor += 1;
            //let val: u8 = payload + cursor;
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
        let data4: i16 =
            Self::try_read_i16_at_cursor(msg, &mut cursor).unwrap_or(0);
        let data5: i16 =
            Self::try_read_i16_at_cursor(msg, &mut cursor).unwrap_or(0);

        (cursor, feature_report_id, data1, data2, data3, data4, data5)
    }

    fn handle_sensor_report_update(&mut self, report_id: u8, timestamp: u128) {
        self.report_update_time[report_id as usize] = timestamp;
        for (_, val) in self.report_update_callbacks[report_id as usize].iter()
        {
            val(self);
        }
    }

    /// Handle parsing of an input report packet,
    /// which may include multiple input reports
    fn handle_sensor_reports(&mut self, received_len: usize) {
        // Sensor input packets have the form:
        // [u8; 5]  timestamp in microseconds for the packet?
        // a sequence of n reports, each with four byte header
        // u8 report ID
        // u8 sequence number of report

        // let mut report_count = 0;
        let mut outer_cursor: usize = PACKET_HEADER_LENGTH + 5; //skip header, timestamp
                                                                //TODO need to skip more above for a payload-level timestamp??
        if received_len < outer_cursor {
            return;
        }

        let payload_len = received_len - outer_cursor;
        if payload_len < 10 {
            log!(
                "bad report: {:?}",
                &self.packet_recv_buf[..PACKET_HEADER_LENGTH]
            );

            return;
        }

        // there may be multiple reports per payload
        while outer_cursor < payload_len {
            //let start_cursor = outer_cursor;
            let (inner_cursor, report_id, data1, data2, data3, data4, data5) =
                Self::handle_one_input_report(
                    outer_cursor,
                    &self.packet_recv_buf[..received_len],
                );
            outer_cursor = inner_cursor;
            // report_count += 1;
            let timestamp: u128;
            match SystemTime::now().duration_since(SystemTime::UNIX_EPOCH) {
                Ok(n) => timestamp = n.as_nanos(),
                Err(_) => timestamp = 0,
            }
            match report_id {
                SENSOR_REPORTID_ACCELEROMETER => {
                    self.update_accelerometer(data1, data2, data3);
                    self.handle_sensor_report_update(report_id, timestamp)
                }
                SENSOR_REPORTID_ROTATION_VECTOR => {
                    self.update_rotation_quaternion(
                        data1, data2, data3, data4, data5,
                    );
                    self.handle_sensor_report_update(report_id, timestamp)
                }
                SENSOR_REPORTID_ROTATION_VECTOR_GAME => {
                    self.update_rotation_quaternion_game(
                        data1, data2, data3, data4,
                    );
                    self.handle_sensor_report_update(report_id, timestamp)
                }
                SENSOR_REPORTID_ROTATION_VECTOR_GEOMAGNETIC => {
                    self.update_rotation_quaternion_geomag(
                        data1, data2, data3, data4, data5,
                    );
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

                _ => {
                    // debug_log!("uhr: {:X}", report_id);
                    // debug_log!("uhr: 0x{:X} {:?}  ", report_id, &self.packet_recv_buf[start_cursor..start_cursor+5]);
                }
            }
        }
    }

    fn update_accelerometer(&mut self, x: i16, y: i16, z: i16) {
        let x = q_to_f32(x, Q_POINTS[SENSOR_REPORTID_ACCELEROMETER as usize]);
        let y = q_to_f32(y, Q_POINTS[SENSOR_REPORTID_ACCELEROMETER as usize]);
        let z = q_to_f32(z, Q_POINTS[SENSOR_REPORTID_ACCELEROMETER as usize]);

        self.accelerometer = [x, y, z];
    }

    /// Given a set of quaternion values in the Q-fixed-point format,
    /// calculate and update the corresponding float values
    fn update_rotation_quaternion(
        &mut self,
        q_i: i16,
        q_j: i16,
        q_k: i16,
        q_r: i16,
        q_a: i16,
    ) {
        //debug_log!("rquat {} {} {} {} {}", q_i, q_j, q_k, q_r, q_a);
        self.rotation_quaternion = [
            q_to_f32(q_i, Q_POINTS[SENSOR_REPORTID_ROTATION_VECTOR as usize]),
            q_to_f32(q_j, Q_POINTS[SENSOR_REPORTID_ROTATION_VECTOR as usize]),
            q_to_f32(q_k, Q_POINTS[SENSOR_REPORTID_ROTATION_VECTOR as usize]),
            q_to_f32(q_r, Q_POINTS[SENSOR_REPORTID_ROTATION_VECTOR as usize]),
        ];
        self.rotation_acc =
            q_to_f32(q_a, Q_POINTS2[SENSOR_REPORTID_ROTATION_VECTOR as usize]);
    }

    /// Given a set of quaternion values in the Q-fixed-point format,
    /// calculate and update the corresponding float values
    fn update_rotation_quaternion_geomag(
        &mut self,
        q_i: i16,
        q_j: i16,
        q_k: i16,
        q_r: i16,
        q_a: i16,
    ) {
        //debug_log!("rquat {} {} {} {} {}", q_i, q_j, q_k, q_r, q_a);
        self.geomag_rotation_quaternion = [
            q_to_f32(
                q_i,
                Q_POINTS[SENSOR_REPORTID_ROTATION_VECTOR_GEOMAGNETIC as usize],
            ),
            q_to_f32(
                q_j,
                Q_POINTS[SENSOR_REPORTID_ROTATION_VECTOR_GEOMAGNETIC as usize],
            ),
            q_to_f32(
                q_k,
                Q_POINTS[SENSOR_REPORTID_ROTATION_VECTOR_GEOMAGNETIC as usize],
            ),
            q_to_f32(
                q_r,
                Q_POINTS[SENSOR_REPORTID_ROTATION_VECTOR_GEOMAGNETIC as usize],
            ),
        ];
        self.geomag_rotation_acc = q_to_f32(
            q_a,
            Q_POINTS2[SENSOR_REPORTID_ROTATION_VECTOR_GEOMAGNETIC as usize],
        );
    }

    /// Given a set of quaternion values in the Q-fixed-point format,
    /// calculate and update the corresponding float values
    fn update_rotation_quaternion_game(
        &mut self,
        q_i: i16,
        q_j: i16,
        q_k: i16,
        q_r: i16,
    ) {
        //debug_log!("rquat {} {} {} {} {}", q_i, q_j, q_k, q_r, q_a);
        self.game_rotation_quaternion = [
            q_to_f32(
                q_i,
                Q_POINTS[SENSOR_REPORTID_ROTATION_VECTOR_GAME as usize],
            ),
            q_to_f32(
                q_j,
                Q_POINTS[SENSOR_REPORTID_ROTATION_VECTOR_GAME as usize],
            ),
            q_to_f32(
                q_k,
                Q_POINTS[SENSOR_REPORTID_ROTATION_VECTOR_GAME as usize],
            ),
            q_to_f32(
                q_r,
                Q_POINTS[SENSOR_REPORTID_ROTATION_VECTOR_GAME as usize],
            ),
        ];
    }

    /// Given a set of linear acceleration values in the Q-fixed-point format,
    /// calculate and update the corresponding float values
    fn update_linear_accel(&mut self, x: i16, y: i16, z: i16) {
        let x = q_to_f32(x, Q_POINTS[SENSOR_REPORTID_LINEAR_ACCEL as usize]);
        let y = q_to_f32(y, Q_POINTS[SENSOR_REPORTID_LINEAR_ACCEL as usize]);
        let z = q_to_f32(z, Q_POINTS[SENSOR_REPORTID_LINEAR_ACCEL as usize]);

        self.linear_accel = [x, y, z];
    }

    /// Given a set of gravity values in the Q-fixed-point format,
    /// calculate and update the corresponding float values
    fn update_gravity(&mut self, x: i16, y: i16, z: i16) {
        let x = q_to_f32(x, Q_POINTS[SENSOR_REPORTID_GRAVITY as usize]);
        let y = q_to_f32(y, Q_POINTS[SENSOR_REPORTID_GRAVITY as usize]);
        let z = q_to_f32(z, Q_POINTS[SENSOR_REPORTID_GRAVITY as usize]);

        self.gravity = [x, y, z];
    }

    /// Given a set of linear acceleration values in the Q-fixed-point format,
    /// calculate and update the corresponding float values
    fn update_gyro_calib(&mut self, x: i16, y: i16, z: i16) {
        let x = q_to_f32(x, Q_POINTS[SENSOR_REPORTID_GYROSCOPE as usize]);
        let y = q_to_f32(y, Q_POINTS[SENSOR_REPORTID_GYROSCOPE as usize]);
        let z = q_to_f32(z, Q_POINTS[SENSOR_REPORTID_GYROSCOPE as usize]);

        self.gyro = [x, y, z];
    }

    /// Given a set of linear acceleration values in the Q-fixed-point format,
    /// calculate and update the corresponding float values
    fn update_gyro_uncalib(&mut self, x: i16, y: i16, z: i16) {
        let x =
            q_to_f32(x, Q_POINTS[SENSOR_REPORTID_GYROSCOPE_UNCALIB as usize]);
        let y =
            q_to_f32(y, Q_POINTS[SENSOR_REPORTID_GYROSCOPE_UNCALIB as usize]);
        let z =
            q_to_f32(z, Q_POINTS[SENSOR_REPORTID_GYROSCOPE_UNCALIB as usize]);

        self.uncalib_gryo = [x, y, z];
    }

    /// Given a set of magnetic field values in the Q-fixed-point format,0xF7
    /// calculate and update the corresponding float values
    fn update_magnetic_field_calib(&mut self, x: i16, y: i16, z: i16) {
        let x = q_to_f32(x, Q_POINTS[SENSOR_REPORTID_MAGNETIC_FIELD as usize]);
        let y = q_to_f32(y, Q_POINTS[SENSOR_REPORTID_MAGNETIC_FIELD as usize]);
        let z = q_to_f32(z, Q_POINTS[SENSOR_REPORTID_MAGNETIC_FIELD as usize]);

        self.mag_field = [x, y, z];
    }
    fn _frs_status_to_str(&self, status: &u8) -> &'static str {
        let s: u8 = *status;
        match s {
            FRS_STATUS_WORD_RECIEVED => {"word(s) received"},
            FRS_STATUS_UNRECOGNIZED_FRS_TYPE => {"unrecognized FRS type"},
            FRS_STATUS_BUSY => {"busy"},
            FRS_STATUS_WRITE_COMPLETE => {"write completed"},
            FRS_STATUS_WRITE_READY => {"write mode entered or ready"},
            FRS_STATUS_WRITE_FAILED => {"write failed"},
            FRS_STATUS_DATA_RECV_NOT_IN_WRITE_MODE => {"data received while not in write mode"},
            FRS_STATUS_INVALID_LENGTH => {"invalid length"},
            FRS_STATUS_RECORD_VALID => {"record valid (the complete record passed internal validation checks)"},
            FRS_STATUS_RECORD_INVALID => {"record invalid (the complete record failed internal validation checks)"},
            FRS_STATUS_DEVICE_ERROR => {"device error (DFU flash memory device unavailable)"},
            FRS_STATUS_READONLY => {"record is read only"},
            FRS_STATUS_NO_DATA => {"no FRS status recieved"},
            _ => {"reserved"},
        }
    }
    /// Handle one or more errors sent in response to a command
    fn handle_cmd_resp_error_list(&mut self, received_len: usize) {
        let payload_len = received_len - PACKET_HEADER_LENGTH;
        let payload = &self.packet_recv_buf[PACKET_HEADER_LENGTH..received_len];

        self.error_list_received = true;
        for cursor in 1..payload_len {
            let err: u8 = payload[cursor];
            self.last_error_received = err;

            eprintln!("Error message from bno08x: {:x}", err);
        }
    }

    pub fn handle_received_packet(&mut self, received_len: usize) {
        let mut _rec_len = received_len;
        if _rec_len > PACKET_RECV_BUF_LEN {
            eprintln!(
                "Packet length of {} exceeded the buffer length of {}",
                received_len, PACKET_RECV_BUF_LEN
            );
            _rec_len = PACKET_RECV_BUF_LEN;
        } else if _rec_len < PACKET_HEADER_LENGTH {
            eprintln!(
                "Packet length of {} was ignored. Shorter than header length of {}",
                received_len, PACKET_HEADER_LENGTH
            );
            return;
        }
        let msg = &self.packet_recv_buf[.._rec_len];
        let chan_num = msg[2];
        //let _seq_num =  msg[3];
        let report_id: u8 = if _rec_len > PACKET_HEADER_LENGTH {
            msg[4]
        } else {
            0
        };
        // log!("packet: {:?}", &self.packet_recv_buf[..received_len]);
        self.last_chan_received = chan_num;
        match chan_num {
            CHANNEL_COMMAND => match report_id {
                CMD_RESP_ADVERTISEMENT => {
                    self.handle_advertise_response(_rec_len);
                }
                CMD_RESP_ERROR_LIST => {
                    self.handle_cmd_resp_error_list(_rec_len);
                }
                _ => {
                    self.last_command_chan_rid = report_id;

                    log!("unh cmd: {}", report_id);
                }
            },
            CHANNEL_EXECUTABLE => match report_id {
                EXECUTABLE_DEVICE_RESP_RESET_COMPLETE => {
                    self.device_reset = true;

                    log!("resp_reset {}", 1);
                }
                _ => {
                    self.last_exec_chan_rid = report_id;

                    log!("unh exe: {:x}", report_id);
                }
            },
            CHANNEL_HUB_CONTROL => {
                match report_id {
                    SHUB_COMMAND_RESP => {
                        // 0xF1 / 241
                        let cmd_resp = msg[6];
                        if cmd_resp == SH2_STARTUP_INIT_UNSOLICITED {
                            self.init_received = true;
                        } else if cmd_resp == SH2_INIT_SYSTEM {
                            self.init_received = true;
                        }

                        log!("CMD_RESP: 0x{:X}", cmd_resp);
                    }
                    SHUB_PROD_ID_RESP => {
                        {
                            //let reset_cause = msg[4 + 1];
                            let _sw_vers_major = msg[4 + 2];
                            let _sw_vers_minor = msg[4 + 3];
                            log!(
                                "PID_RESP {}.{}",
                                _sw_vers_major,
                                _sw_vers_major
                            );
                        }

                        self.prod_id_verified = true;
                    }
                    SHUB_GET_FEATURE_RESP => {
                        // 0xFC
                        log!("feat resp: {}", msg[5]);
                        self.report_enabled[msg[5] as usize] = true;
                    }
                    SHUB_FRS_WRITE_RESP => {
                        //0xF5
                        log!(
                            "write resp: {}",
                            self._frs_status_to_str(&msg[5])
                        );
                        let status = msg[5];
                        self.frs_write_status = status
                        // match status {
                        //     0 => {
                        //         println!(
                        //             "Recieved word, offset = {}",
                        //             msg[4 + 2] as u32
                        //                 + ((msg[4 + 3] as u32) << 8)
                        //         )
                        //     }
                        //     3 | 5 => {
                        //         self.frs_write_status = false;
                        //     }
                        //     4 => {
                        //         self.frs_write_status = true;
                        //     }
                        //     _ => {}
                        // }
                    }
                    _ => {
                        log!(
                            "unh hbc: 0x{:X} {:x?}",
                            report_id,
                            &msg[..PACKET_HEADER_LENGTH]
                        );
                    }
                }
            }
            CHANNEL_SENSOR_REPORTS => {
                self.handle_sensor_reports(_rec_len);
            }
            _ => {
                self.last_chan_received = chan_num;

                log!("unh chan 0x{:X}", chan_num);
            }
        }
    }

    /// The BNO080 starts up with all sensors disabled,
    /// waiting for the application to configure it.
    pub fn init(&mut self) -> Result<(), WrapperError<SE>> {
        log!("wrapper init");

        //Section 5.1.1.1 : On system startup, the SHTP control application will send
        // its full advertisement response, unsolicited, to the host.
        delay_ms(1);
        self.sensor_interface
            .setup()
            .map_err(WrapperError::CommError)?;

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
            log!("Eating advertisement response");
            self.handle_one_message(20);
            log!("Eating reset response");
            delay_ms(250);
            self.handle_one_message(20);
            // eat the unsolicited initialization response
            // log!("Eating initialization response");
            // Further reads don't respond if we uncomment this
            // delay_ms(255);
            // self.handle_one_message(255);
        }
        self.verify_product_id()?;
        delay_ms(100);
        Ok(())
    }

    /// Tell the sensor to start reporting the fused rotation vector
    /// on a regular cadence. Note that the maximum valid update rate
    /// is 1 kHz, based on the max update rate of the sensor's gyros.
    /// Returns True if the report was successfully enabled. Otherwise returns False.
    pub fn enable_rotation_vector(
        &mut self,
        millis_between_reports: u16,
    ) -> Result<bool, WrapperError<SE>> {
        self.enable_report(
            SENSOR_REPORTID_ROTATION_VECTOR,
            millis_between_reports,
        )
    }

    /// Enables reporting of linear acceleration vector. Returns True if the report was successfully enabled. Otherwise returns False.
    pub fn enable_linear_accel(
        &mut self,
        millis_between_reports: u16,
    ) -> Result<bool, WrapperError<SE>> {
        self.enable_report(SENSOR_REPORTID_LINEAR_ACCEL, millis_between_reports)
    }

    /// Enables reporting of gyroscope data. Returns True if the report was successfully enabled. Otherwise returns False.
    pub fn enable_gyro(
        &mut self,
        millis_between_reports: u16,
    ) -> Result<bool, WrapperError<SE>> {
        self.enable_report(SENSOR_REPORTID_GYROSCOPE, millis_between_reports)
    }

    /// Enables reporting of linear acceleration vector. Returns True if the report was successfully enabled. Otherwise returns False.
    pub fn enable_gravity(
        &mut self,
        millis_between_reports: u16,
    ) -> Result<bool, WrapperError<SE>> {
        self.enable_report(SENSOR_REPORTID_GRAVITY, millis_between_reports)
    }
    pub fn report_update_time(&self, report_id: u8) -> u128 {
        if report_id as usize <= self.report_enabled.len() {
            return self.report_update_time[report_id as usize];
        }
        0
    }
    pub fn is_report_enabled(&self, report_id: u8) -> bool {
        if report_id as usize <= self.report_enabled.len() {
            return self.report_enabled[report_id as usize];
        }
        false
    }

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

    pub fn remove_sensor_report_callback(
        &mut self,
        report_id: u8,
        key: String,
    ) {
        self.report_update_callbacks[report_id as usize].remove(&key);
    }
    /// Enable a particular report. Returns True if the report was successfully enabled. Otherwise returns False.
    pub fn enable_report(
        &mut self,
        report_id: u8,
        millis_between_reports: u16,
    ) -> Result<bool, WrapperError<SE>> {
        log!("enable_report 0x{:X}", report_id);

        let micros_between_reports: u32 =
            (millis_between_reports as u32) * 1000;
        let cmd_body: [u8; 17] = [
            SHUB_REPORT_SET_FEATURE_CMD,
            report_id,
            0,                                        //feature flags
            0,                                        //LSB change sensitivity
            0,                                        //MSB change sensitivity
            (micros_between_reports & 0xFFu32) as u8, // LSB report interval, microseconds
            (micros_between_reports.shr(8) & 0xFFu32) as u8,
            (micros_between_reports.shr(16) & 0xFFu32) as u8,
            (micros_between_reports.shr(24) & 0xFFu32) as u8, // MSB report interval
            0, // LSB Batch Interval
            0,
            0,
            0, // MSB Batch interval
            0, // LSB sensor-specific config
            0,
            0,
            0, // MSB sensor-specific config
        ];
        //we simply blast out this configuration command and assume it'll succeed
        // let _size = self.send_packet(CHANNEL_HUB_CONTROL, &cmd_body)?;
        self.send_packet(CHANNEL_HUB_CONTROL, &cmd_body)?;
        // any error or success in configuration will arrive some time later
        let start = Instant::now();
        while !self.report_enabled[report_id as usize]
            && start.elapsed().as_millis() < 2000
        {
            let rc = self.receive_packet_with_timeout(250);
            if rc.is_ok() {
                let received_len = rc.unwrap();
                if received_len > 0 {
                    self.handle_received_packet(received_len);
                }
            }
        }
        delay_ms(200);
        log!(
            "Report {:x} is enabled: {}",
            report_id,
            self.report_enabled[report_id as usize]
        );
        if !self.report_enabled[report_id as usize] {
            // eprintln!("Could not enable report id: {}", report_id);
            return Ok(false);
        }
        Ok(true)
    }

    pub fn set_sensor_orientation(
        &mut self,
        qi: f32,
        qj: f32,
        qk: f32,
        qr: f32,
        timeout: u128,
    ) -> Result<bool, WrapperError<SE>> {
        let length: u16 = 4;
        let frs_type: u16 = 0x2D3E;
        let cmd_body_req: [u8; 6] = [
            SHUB_FRS_WRITE_REQ,      //request product ID
            0,                       //reserved
            (length & 0xFF) as u8,   // length LSB
            length.shr(8) as u8,     // length MSB
            (frs_type & 0xFF) as u8, // FRS Type LSB
            frs_type.shr(8) as u8,   // FRS Type MSB
        ];
        let _ = self.send_packet(CHANNEL_HUB_CONTROL, cmd_body_req.as_ref())?;
        self.frs_write_status = FRS_STATUS_NO_DATA;
        let mut start = Instant::now();
        while self.frs_write_status == FRS_STATUS_NO_DATA
            && start.elapsed().as_millis() < timeout
        {
            let rc = self.receive_packet_with_timeout(250);
            if rc.is_ok() {
                let received_len = rc.unwrap();
                if received_len > 0 {
                    self.handle_received_packet(received_len);
                }
            }
        }

        if self.frs_write_status != FRS_STATUS_WRITE_READY {
            log!("FRS Write not ready");
            return Ok(false);
        }
        log!("FRS Write ready");
        delay_ms(150);
        let mut offset: u16 = 0;
        let q30_qi = f32_to_q(qi, 30);
        let q30_qj = f32_to_q(qj, 30);
        let q30_qk = f32_to_q(qk, 30);
        let q30_qr = f32_to_q(qr, 30);
        let mut cmd_body_data: [u8; 12] = [
            SHUB_FRS_WRITE_DATA_REQ, //request product ID
            0,                       //reserved
            (offset & 0xFF) as u8,   // length LSB
            offset.shr(8) as u8,     // length MSB
            q30_qi[0],               // FRS data0 LSB
            q30_qi[1],               //
            q30_qi[2],               //
            q30_qi[3],               // FRS data1 MSB
            q30_qj[0],               // FRS data0 LSB
            q30_qj[1],               //
            q30_qj[2],               //
            q30_qj[3],               // FRS data1 MSB
        ];
        _ = self.send_packet(CHANNEL_HUB_CONTROL, cmd_body_data.as_ref())?;

        self.frs_write_status = FRS_STATUS_NO_DATA;
        start = Instant::now();
        while self.frs_write_status == FRS_STATUS_NO_DATA
            && start.elapsed().as_millis() < 800
        {
            let rc = self.receive_packet_with_timeout(250);
            if rc.is_ok() {
                let received_len = rc.unwrap();
                if received_len > 0 {
                    self.handle_received_packet(received_len);
                }
            }
        }
        delay_ms(150);
        offset += 2;
        cmd_body_data = [
            SHUB_FRS_WRITE_DATA_REQ, //request product ID
            0,                       //reserved
            (offset & 0xFF) as u8,   // length LSB
            offset.shr(8) as u8,     // length MSB
            q30_qk[0],               // FRS data0 LSB
            q30_qk[1],               //
            q30_qk[2],               //
            q30_qk[3],               // FRS data1 MSB
            q30_qr[0],               // FRS data0 LSB
            q30_qr[1],               //
            q30_qr[2],               //
            q30_qr[3],               // FRS data1 MSB
        ];
        _ = self.send_packet(CHANNEL_HUB_CONTROL, cmd_body_data.as_ref())?;
        self.frs_write_status = FRS_STATUS_NO_DATA;
        start = Instant::now();
        while self.frs_write_status != FRS_STATUS_WRITE_FAILED
            && self.frs_write_status != FRS_STATUS_WRITE_COMPLETE
            && start.elapsed().as_millis() < 800
        {
            let rc = self.receive_packet_with_timeout(250);
            if rc.is_ok() {
                let received_len = rc.unwrap();
                if received_len > 0 {
                    self.handle_received_packet(received_len);
                }
            }
        }
        delay_ms(100);
        if self.frs_write_status == FRS_STATUS_WRITE_COMPLETE {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Prepare a packet for sending, in our send buffer
    fn prep_send_packet(&mut self, channel: u8, body_data: &[u8]) -> usize {
        let body_len = body_data.len();

        let packet_length = body_len + PACKET_HEADER_LENGTH;
        let packet_header = [
            (packet_length & 0xFF) as u8, //LSB
            packet_length.shr(8) as u8,   //MSB
            channel,
            self.sequence_numbers[channel as usize],
        ];
        self.sequence_numbers[channel as usize] += 1;

        self.packet_send_buf[..PACKET_HEADER_LENGTH]
            .copy_from_slice(packet_header.as_ref());
        self.packet_send_buf[PACKET_HEADER_LENGTH..packet_length]
            .copy_from_slice(body_data);

        packet_length
    }

    /// Send packet from our packet send buf
    fn send_packet(
        &mut self,
        channel: u8,
        body_data: &[u8],
    ) -> Result<usize, WrapperError<SE>> {
        let packet_length = self.prep_send_packet(channel, body_data);
        // log!("Sending {:?}", &self.packet_send_buf[..packet_length]);

        let rc = self
            .sensor_interface
            .send_and_receive_packet(
                &self.packet_send_buf[..packet_length],
                &mut self.packet_recv_buf,
            )
            .map_err(WrapperError::CommError)?;
        if rc > 0 {
            self.handle_received_packet(rc);
        }
        Ok(packet_length)
    }

    /// Read one packet into the receive buffer
    pub(crate) fn receive_packet_with_timeout(
        &mut self,
        max_ms: usize,
    ) -> Result<usize, WrapperError<SE>> {
        self.packet_recv_buf[0] = 0;
        self.packet_recv_buf[1] = 0;
        let packet_len = self
            .sensor_interface
            .read_with_timeout(&mut self.packet_recv_buf, max_ms)
            .map_err(WrapperError::CommError)?;

        self.last_packet_len_received = packet_len;

        Ok(packet_len)
    }

    /// Verify that the sensor returns an expected chip ID
    fn verify_product_id(&mut self) -> Result<(), WrapperError<SE>> {
        log!("request PID...");
        let cmd_body: [u8; 2] = [
            SHUB_PROD_ID_REQ, //request product ID
            0,                //reserved
        ];

        // for some reason, reading PID right sending request does not work with i2c
        if self.sensor_interface.requires_soft_reset() {
            self.send_packet(CHANNEL_HUB_CONTROL, cmd_body.as_ref())?;
        } else {
            let response_size = self.send_and_receive_packet(
                CHANNEL_HUB_CONTROL,
                cmd_body.as_ref(),
            )?;
            if response_size > 0 {
                self.handle_received_packet(response_size);
            }
        };

        // process all incoming messages until we get a product id (or no more data)
        while !self.prod_id_verified {
            log!("read PID");
            let msg_count = self.handle_one_message(150);
            if msg_count < 1 {
                break;
            }
        }

        if !self.prod_id_verified {
            return Err(WrapperError::InvalidChipId(0));
        }
        Ok(())
    }

    pub fn accelerometer(&self) -> Result<[f32; 3], WrapperError<SE>> {
        Ok(self.accelerometer)
    }

    /// Read normalized quaternion:
    /// QX normalized quaternion – X | range: 0.0 – 1.0 ( ±π )
    /// QY normalized quaternion – Y | range: 0.0 – 1.0 ( ±π/2 )
    /// QZ normalized quaternion – Z | range: 0.0 – 1.0 ( ±π )
    /// QW normalized quaternion – W | range: 0.0 – 1.0
    pub fn rotation_quaternion(&self) -> Result<[f32; 4], WrapperError<SE>> {
        Ok(self.rotation_quaternion)
    }

    pub fn rotation_acc(&self) -> f32 {
        self.rotation_acc
    }

    /// Read normalized quaternion:
    /// QX normalized quaternion – X | range: 0.0 – 1.0 ( ±π )
    /// QY normalized quaternion – Y | range: 0.0 – 1.0 ( ±π/2 )
    /// QZ normalized quaternion – Z | range: 0.0 – 1.0 ( ±π )
    /// QW normalized quaternion – W | range: 0.0 – 1.0
    pub fn game_rotation_quaternion(
        &self,
    ) -> Result<[f32; 4], WrapperError<SE>> {
        Ok(self.game_rotation_quaternion)
    }

    /// Read normalized quaternion:
    /// QX normalized quaternion – X | range: 0.0 – 1.0 ( ±π )
    /// QY normalized quaternion – Y | range: 0.0 – 1.0 ( ±π/2 )
    /// QZ normalized quaternion – Z | range: 0.0 – 1.0 ( ±π )
    /// QW normalized quaternion – W | range: 0.0 – 1.0
    pub fn geomag_rotation_quaternion(
        &self,
    ) -> Result<[f32; 4], WrapperError<SE>> {
        Ok(self.rotation_quaternion)
    }

    pub fn geomag_rotation_acc(&self) -> f32 {
        self.geomag_rotation_acc
    }
    /// Read linear acceleration (m/s^2)
    pub fn linear_accel(&self) -> Result<[f32; 3], WrapperError<SE>> {
        Ok(self.linear_accel)
    }

    /// Read gravity (m/s^2)
    pub fn gravity(&self) -> Result<[f32; 3], WrapperError<SE>> {
        Ok(self.gravity)
    }

    /// Read calibrated gyroscope data (rad/s)
    pub fn gyro(&self) -> Result<[f32; 3], WrapperError<SE>> {
        Ok(self.gyro)
    }

    /// Read uncalibrated gyroscope data (rad/s)
    pub fn gyro_uncalib(&self) -> Result<[f32; 3], WrapperError<SE>> {
        Ok(self.uncalib_gryo)
    }

    /// Read magnetic field data (uT)
    pub fn mag_field(&self) -> Result<[f32; 3], WrapperError<SE>> {
        Ok(self.mag_field)
    }

    /// Tell the sensor to reset.
    /// Normally applications should not need to call this directly,
    /// as it is called during `init`.
    pub fn soft_reset(&mut self) -> Result<(), WrapperError<SE>> {
        //
        log!("soft_reset");
        let data: [u8; 1] = [EXECUTABLE_DEVICE_CMD_RESET];
        // send command packet and ignore received packets
        let received_len =
            self.send_and_receive_packet(CHANNEL_EXECUTABLE, data.as_ref())?;
        if received_len > 0 {
            self.handle_received_packet(received_len);
        }

        Ok(())
    }

    /// Send a packet and receive the response
    fn send_and_receive_packet(
        &mut self,
        channel: u8,
        body_data: &[u8],
    ) -> Result<usize, WrapperError<SE>> {
        let send_packet_length = self.prep_send_packet(channel, body_data);

        let recv_packet_length = self
            .sensor_interface
            .send_and_receive_packet(
                self.packet_send_buf[..send_packet_length].as_ref(),
                &mut self.packet_recv_buf,
            )
            .map_err(WrapperError::CommError)?;

        Ok(recv_packet_length)
    }
}

fn q_to_f32(q_val: i16, q_point: usize) -> f32 {
    (q_val as f32) / (1.shl(q_point) as f32)
}

fn f32_to_q(f32_val: f32, q_point: usize) -> [u8; 4] {
    ((f32_val as f64 * (1.shl(q_point) as f64)) as i32).to_le_bytes()
}
// The BNO080 supports six communication channels:
const CHANNEL_COMMAND: u8 = 0;
/// the SHTP command channel
const CHANNEL_EXECUTABLE: u8 = 1;
/// executable channel
const CHANNEL_HUB_CONTROL: u8 = 2;
/// sensor hub control channel
const CHANNEL_SENSOR_REPORTS: u8 = 3;
/// input sensor reports (non-wake, not gyroRV)
//const  CHANNEL_WAKE_REPORTS: usize = 4; /// wake input sensor reports (for sensors configured as wake up sensors)
//const  CHANNEL_GYRO_ROTATION: usize = 5; ///  gyro rotation vector (gyroRV)

/// Command Channel requests / responses

// Commands
//const CMD_GET_ADVERTISEMENT: u8 = 0;
//const CMD_SEND_ERROR_LIST: u8 = 1;

/// Responses
const CMD_RESP_ADVERTISEMENT: u8 = 0;
const CMD_RESP_ERROR_LIST: u8 = 1;

/// SHTP constants

/// Report ID for Product ID request
const SHUB_PROD_ID_REQ: u8 = 0xF9;
/// Report ID for Product ID response
const SHUB_PROD_ID_RESP: u8 = 0xF8;
const SHUB_GET_FEATURE_RESP: u8 = 0xFC;
const SHUB_REPORT_SET_FEATURE_CMD: u8 = 0xFD;
// const SHUB_GET_FEATURE_REQ: u8 = 0xFE;
// const SHUB_FORCE_SENSOR_FLUSH: u8 = 0xF0;
const SHUB_COMMAND_RESP: u8 = 0xF1;
//const SHUB_COMMAND_REQ:u8 =  0xF2;
const SHUB_FRS_WRITE_REQ: u8 = 0xF7;
const SHUB_FRS_WRITE_DATA_REQ: u8 = 0xF6;
const SHUB_FRS_WRITE_RESP: u8 = 0xF5;

// some mysterious responses we sometimes get:
// 0x78, 0x7C

/// Report IDs from SH2 Reference Manual:
// 0x01 accelerometer (m/s^2 including gravity): Q point 8
pub const SENSOR_REPORTID_ACCELEROMETER: u8 = 0x01;
// 0x02 gyroscope calibrated (rad/s): Q point 9
pub const SENSOR_REPORTID_GYROSCOPE: u8 = 0x02;

// 0x03 mag field calibrated (uTesla): Q point 4
pub const SENSOR_REPORTID_MAGNETIC_FIELD: u8 = 0x03;

/// Linear acceleration (m/s^2 minus gravity): Q point 8
pub const SENSOR_REPORTID_LINEAR_ACCEL: u8 = 0x04;

/// Unit quaternion rotation vector, Q point 14, with heading accuracy estimate (radians) Q point 12
pub const SENSOR_REPORTID_ROTATION_VECTOR: u8 = 0x05;
pub const SENSOR_REPORTID_GRAVITY: u8 = 0x06; // Q point 8
/// Gyroscope uncalibrated (rad/s): Q point 9
pub const SENSOR_REPORTID_GYROSCOPE_UNCALIB: u8 = 0x07;
// 0x08 game rotation vector : Q point 14
pub const SENSOR_REPORTID_ROTATION_VECTOR_GAME: u8 = 0x08;
// 0x09 geomagnetic rotation vector: Q point 14 for quaternion, Q point 12 for heading accuracy
pub const SENSOR_REPORTID_ROTATION_VECTOR_GEOMAGNETIC: u8 = 0x09;

// 0x0A pressure (hectopascals) from external baro: Q point 20
// 0x0B ambient light (lux) from external sensor: Q point 8
// 0x0C humidity (percent) from external sensor: Q point 8
// 0x0D proximity (centimeters) from external sensor: Q point 4
// 0x0E temperature (degrees C) from external sensor: Q point 7

pub const FRS_STATUS_WORD_RECIEVED: u8 = 0;
pub const FRS_STATUS_UNRECOGNIZED_FRS_TYPE: u8 = 1;
pub const FRS_STATUS_BUSY: u8 = 2;
pub const FRS_STATUS_WRITE_COMPLETE: u8 = 3;
pub const FRS_STATUS_WRITE_READY: u8 = 4;
pub const FRS_STATUS_WRITE_FAILED: u8 = 5;
pub const FRS_STATUS_DATA_RECV_NOT_IN_WRITE_MODE: u8 = 6;
pub const FRS_STATUS_INVALID_LENGTH: u8 = 7;
pub const FRS_STATUS_RECORD_VALID: u8 = 8;
pub const FRS_STATUS_RECORD_INVALID: u8 = 9;
pub const FRS_STATUS_DEVICE_ERROR: u8 = 10;
pub const FRS_STATUS_READONLY: u8 = 11;
pub const FRS_STATUS_RESERVED: u8 = 12;
pub const FRS_STATUS_NO_DATA: u8 = u8::MAX;

const Q_POINTS: [usize; 15] = [0, 8, 9, 4, 8, 14, 8, 9, 14, 14, 0, 0, 0, 0, 0];
const Q_POINTS2: [usize; 15] = [0, 0, 0, 0, 0, 12, 0, 0, 0, 12, 0, 0, 0, 0, 0];

/// executable/device channel responses
/// Figure 1-27: SHTP executable commands and response
// const EXECUTABLE_DEVICE_CMD_UNKNOWN: u8 =  0;
const EXECUTABLE_DEVICE_CMD_RESET: u8 = 1;
//const EXECUTABLE_DEVICE_CMD_ON: u8 =   2;
//const EXECUTABLE_DEVICE_CMD_SLEEP =  3;

/// Response to CMD_RESET
const EXECUTABLE_DEVICE_RESP_RESET_COMPLETE: u8 = 1;

/// Commands and subcommands
const SH2_INIT_UNSOLICITED: u8 = 0x80;
const SH2_CMD_INITIALIZE: u8 = 4;
const SH2_INIT_SYSTEM: u8 = 1;
const SH2_STARTUP_INIT_UNSOLICITED: u8 =
    SH2_CMD_INITIALIZE | SH2_INIT_UNSOLICITED;

#[cfg(test)]
mod tests {
    // use super::*;
    use crate::interface::delay::TimerMs;
    use crate::wrapper::f32_to_q;
    use crate::wrapper::BNO08x;

    #[test]
    fn test_f32_to_q() {
        assert_eq!(
            [0x10, 0x00, 0x00, 0x00],
            f32_to_q(0.25, 30),
            "Wrong positive q point value"
        );
        assert_eq!(
            [0xf0, 0x00, 0x00, 0x00],
            f32_to_q(-0.25, 30),
            "Wrong negative q point value"
        );
    }

    // Actual advertising packet received from sensor:
    pub const ADVERTISING_PACKET_FULL: [u8; 276] = [
        0x14, 0x81, 0x00, 0x01, 0x00, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x80,
        0x06, 0x31, 0x2e, 0x30, 0x2e, 0x30, 0x00, 0x02, 0x02, 0x00, 0x01, 0x03,
        0x02, 0xff, 0x7f, 0x04, 0x02, 0x00, 0x01, 0x05, 0x02, 0xff, 0x7f, 0x08,
        0x05, 0x53, 0x48, 0x54, 0x50, 0x00, 0x06, 0x01, 0x00, 0x09, 0x08, 0x63,
        0x6f, 0x6e, 0x74, 0x72, 0x6f, 0x6c, 0x00, 0x01, 0x04, 0x01, 0x00, 0x00,
        0x00, 0x08, 0x0b, 0x65, 0x78, 0x65, 0x63, 0x75, 0x74, 0x61, 0x62, 0x6c,
        0x65, 0x00, 0x06, 0x01, 0x01, 0x09, 0x07, 0x64, 0x65, 0x76, 0x69, 0x63,
        0x65, 0x00, 0x01, 0x04, 0x02, 0x00, 0x00, 0x00, 0x08, 0x0a, 0x73, 0x65,
        0x6e, 0x73, 0x6f, 0x72, 0x68, 0x75, 0x62, 0x00, 0x06, 0x01, 0x02, 0x09,
        0x08, 0x63, 0x6f, 0x6e, 0x74, 0x72, 0x6f, 0x6c, 0x00, 0x06, 0x01, 0x03,
        0x09, 0x0c, 0x69, 0x6e, 0x70, 0x75, 0x74, 0x4e, 0x6f, 0x72, 0x6d, 0x61,
        0x6c, 0x00, 0x07, 0x01, 0x04, 0x09, 0x0a, 0x69, 0x6e, 0x70, 0x75, 0x74,
        0x57, 0x61, 0x6b, 0x65, 0x00, 0x06, 0x01, 0x05, 0x09, 0x0c, 0x69, 0x6e,
        0x70, 0x75, 0x74, 0x47, 0x79, 0x72, 0x6f, 0x52, 0x76, 0x00, 0x80, 0x06,
        0x31, 0x2e, 0x31, 0x2e, 0x30, 0x00, 0x81, 0x64, 0xf8, 0x10, 0xf5, 0x04,
        0xf3, 0x10, 0xf1, 0x10, 0xfb, 0x05, 0xfa, 0x05, 0xfc, 0x11, 0xef, 0x02,
        0x01, 0x0a, 0x02, 0x0a, 0x03, 0x0a, 0x04, 0x0a, 0x05, 0x0e, 0x06, 0x0a,
        0x07, 0x10, 0x08, 0x0c, 0x09, 0x0e, 0x0a, 0x08, 0x0b, 0x08, 0x0c, 0x06,
        0x0d, 0x06, 0x0e, 0x06, 0x0f, 0x10, 0x10, 0x05, 0x11, 0x0c, 0x12, 0x06,
        0x13, 0x06, 0x14, 0x10, 0x15, 0x10, 0x16, 0x10, 0x17, 0x00, 0x18, 0x08,
        0x19, 0x06, 0x1a, 0x00, 0x1b, 0x00, 0x1c, 0x06, 0x1d, 0x00, 0x1e, 0x10,
        0x1f, 0x00, 0x20, 0x00, 0x21, 0x00, 0x22, 0x00, 0x23, 0x00, 0x24, 0x00,
        0x25, 0x00, 0x26, 0x00, 0x27, 0x00, 0x28, 0x0e, 0x29, 0x0c, 0x2a, 0x0e,
    ];
}
