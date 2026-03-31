// Copyright 2025 Au-Zone Technologies Inc.
// SPDX-License-Identifier: Apache-2.0

use log::{error, trace};

use super::SensorInterface;
use crate::{
    interface::{
        delay::delay_ms,
        gpio::{InputPin, OutputPin},
        spidev::{Read, Transfer, Write},
        SensorCommon, PACKET_HEADER_LENGTH,
    },
    Error::{self, BufferOverflow, NoDataAvailable, SensorUnresponsive},
};
use std::{
    fmt::Debug,
    time::{Duration, Instant, SystemTime},
};

/// Encapsulates all the lines required to operate this sensor
/// - SCK: clock line from master
/// - MISO: Data input from the sensor to the master
/// - MOSI: Output from the master to the sensor
/// - CSN: chip select line that selects the device on the shared SPI bus
/// - HINTN: Hardware Interrupt. Sensor uses this to indicate it had data
///   available for read
/// - RSTN: Reset the device
pub struct SpiControlLines<SPI, /* CSN, */ IN, RSTN> {
    pub spi: SPI, // the spidev read/write
    // pub csn: CSN,    // chip select pin, SPI_CS
    pub hintn: IN,   // interrupt, IMU_INT
    pub reset: RSTN, // reset, IMU_RST
}

/// This combines the SPI peripheral and associated control pins
pub struct SpiInterface<SPI, /* CSN, */ IN, RSTN> {
    spi: SPI,
    // csn: CSN,
    hintn: IN,
    reset: RSTN,
    received_packet_count: usize,
}

impl<SPI, /* CSN, */ IN, RSTN, CommE, PinE> SpiInterface<SPI, /* CSN, */ IN, RSTN>
where
    SPI: Write<Error = CommE> + Transfer<Error = CommE> + Read<Error = CommE>,
    // CSN: OutputPin<Error = PinE>,
    IN: InputPin<Error = PinE>,
    RSTN: OutputPin<Error = PinE>,
    CommE: core::fmt::Debug,
    PinE: core::fmt::Debug,
{
    pub fn new(lines: SpiControlLines<SPI, /* CSN, */ IN, RSTN>) -> Self {
        Self {
            spi: lines.spi,
            // csn: lines.csn,
            hintn: lines.hintn,
            reset: lines.reset,
            received_packet_count: 0,
        }
    }

    /// Is the sensor indicating it has data available
    /// "In SPI and I2C mode the HOST_INTN signal is used by the BNO080 to
    /// indicate to the application processor that the BNO080 needs attention."
    fn hintn_signaled(&self) -> bool {
        self.hintn.is_low().unwrap_or(false)
    }

    /// Wait for sensor to be ready after a reset.
    /// After reset this can take around 120 ms
    /// Return true if the sensor is awake, false if it doesn't wake up
    /// `max_ms` maximum milliseconds to await for HINTN change
    fn wait_for_sensor_awake(&mut self, max_ms: usize) -> bool {
        let mut low_detected = false;
        let start = Instant::now();
        while start.elapsed() < Duration::from_millis(max_ms as u64) {
            if !self.hintn_signaled() {
                low_detected = true;
            }

            if low_detected && self.hintn_signaled() {
                return true;
            }
        }

        false
    }

    /// Return true if hintn was signaled within `max_ms` milliseconds, false
    /// otherwise
    fn block_on_hintn(&mut self, max_ms: usize) -> bool {
        if self.hintn_signaled() {
            return true;
        }
        let deadline = Instant::now() + Duration::from_millis(max_ms as u64);
        while let Some(rem) = deadline.checked_duration_since(Instant::now()) {
            if self.hintn.read_event_with_timeout(rem).unwrap().is_some() {
                if self.hintn_signaled() {
                    return true;
                }
            } else {
                return false;
            }
        }
        false
    }

    /// Similar to block_on_hintn but without a timeout. Returns true if hintn
    /// was signaled, false if the hintn event stream was disconnected
    /// This function can block forever if the sensor never signals hintn, so
    /// use with caution
    fn block_on_hintn_no_limit(&mut self) -> bool {
        if self.hintn_signaled() {
            return true;
        }
        loop {
            if self.hintn.read_event().is_ok() && self.hintn_signaled() {
                return true;
            }
        }
    }

    /// Assumes hintn has already been waited on and signaled, attempts to read
    /// a packet immediately without waiting for hintn again.
    fn read_packet_immediately(
        &mut self,
        recv_buf: &mut [u8],
    ) -> Result<usize, Error<CommE, PinE>> {
        // check how long the message to read is
        let mut read_packet_len = 0;
        recv_buf[..PACKET_HEADER_LENGTH].fill(0);

        let rc = self.spi.read(&mut recv_buf[..PACKET_HEADER_LENGTH]);
        if rc.is_ok() {
            read_packet_len = SensorCommon::parse_packet_header(&recv_buf[..PACKET_HEADER_LENGTH]);
        }

        //zero the receive buffer
        recv_buf[..read_packet_len].fill(0);

        if !self.block_on_hintn_no_limit() {
            error!("No message to read - HINTN Err");
            return Err(NoDataAvailable);
        }

        let rc = self.spi.read(&mut recv_buf[..read_packet_len]);
        if rc.is_ok() {
            read_packet_len = SensorCommon::parse_packet_header(&recv_buf[..PACKET_HEADER_LENGTH]);
        }

        if read_packet_len > 0 {
            self.received_packet_count += 1;
        }

        Ok(read_packet_len)
    }
}

impl<SPI, /* CSN, */ IN, RS, CommE, PinE> SensorInterface
    for SpiInterface<SPI, /* CSN, */ IN, RS>
where
    SPI: Write<Error = CommE> + Transfer<Error = CommE> + Read<Error = CommE>,
    // CSN: OutputPin<Error = PinE>,
    IN: InputPin<Error = PinE>,
    RS: OutputPin<Error = PinE>,
    CommE: Debug,
    PinE: Debug,
{
    type SensorError = Error<CommE, PinE>;

    fn requires_soft_reset(&self) -> bool {
        false
    }

    fn setup(&mut self) -> Result<(), Self::SensorError> {
        // Deselect sensor
        // self.csn.set_high().map_err(Error::Pin)?;
        // Note: This assumes that WAK/PS0 is set to high already

        self.reset.set_high().map_err(Error::Pin)?;

        trace!("reset cycle... ");
        // reset cycle
        delay_ms(2);
        self.reset.set_low().map_err(Error::Pin)?;
        delay_ms(2);
        self.reset.set_high().map_err(Error::Pin)?;

        // wait for sensor to set hintn pin after reset
        let ready = self.wait_for_sensor_awake(200);
        if !ready {
            return Err(SensorUnresponsive);
        }

        Ok(())
    }

    fn send_and_receive_packet(
        &mut self,
        send_buf: &[u8],
        recv_buf: &mut [u8],
    ) -> Result<usize, Self::SensorError> {
        if send_buf.len() > recv_buf.len() {
            error!(
                "Send buffer length ({}) greater than receive buffer length ({})",
                send_buf.len(),
                recv_buf.len()
            );
            return Err(BufferOverflow {
                packet_size: send_buf.len(),
                buffer_size: recv_buf.len(),
            });
        }

        //zero the receive buffer
        recv_buf.fill(0);

        // check how long the message to read is
        let mut read_packet_len = 0;

        // According to diagram, does not have PS0/WAKE pin connected to GPIO, so we
        // will need to wait for hintn pin to write and cannot use the WAKE pin
        // to indicate we want to write. https://au-zone.atlassian.net/browse/TOP2-188
        // MVN2-300000 R00A GNSS-IMU Schematics.PDF
        if !self.block_on_hintn(100) {
            error!("No message to read - HINTN timeout");
            return Err(NoDataAvailable);
        }

        let rc = self.spi.transfer(&mut recv_buf[..PACKET_HEADER_LENGTH]);
        if rc.is_ok() {
            read_packet_len = SensorCommon::parse_packet_header(&recv_buf[..PACKET_HEADER_LENGTH]);
        }

        // Copy the write message into the buffer
        recv_buf[..send_buf.len()].copy_from_slice(send_buf);
        let total_packet_len = std::cmp::max(read_packet_len, send_buf.len());
        if total_packet_len > recv_buf.len() {
            error!(
                "Total packet length ({}) greater than recv buffer size ({})",
                total_packet_len,
                recv_buf.len()
            );
            return Err(BufferOverflow {
                packet_size: total_packet_len,
                buffer_size: recv_buf.len(),
            });
        }

        if !self.block_on_hintn(100) {
            error!("No message to read - HINTN timeout");
            return Err(NoDataAvailable);
        }

        let rc = self.spi.transfer(&mut recv_buf[..total_packet_len]);
        if rc.is_ok() {
            read_packet_len = SensorCommon::parse_packet_header(&recv_buf[..PACKET_HEADER_LENGTH]);
        }

        if read_packet_len > 0 {
            self.received_packet_count += 1;
        }
        Ok(read_packet_len)
    }

    fn write_packet(&mut self, packet: &[u8]) -> Result<(), Self::SensorError> {
        self.spi.write(packet).map_err(Error::Comm)?;
        Ok(())
    }

    /// Read a complete packet from the sensor
    fn read_packet(&mut self, recv_buf: &mut [u8]) -> Result<usize, Self::SensorError> {
        if !self.block_on_hintn_no_limit() {
            error!("No message to read - HINTN Err");
            return Err(NoDataAvailable);
        }
        self.read_packet_immediately(recv_buf)
    }

    fn read_with_timeout(
        &mut self,
        recv_buf: &mut [u8],
        max_ms: usize,
    ) -> Result<(usize, SystemTime), Self::SensorError> {
        if self.block_on_hintn(max_ms) {
            let timestamp = SystemTime::now();
            return Ok((self.read_packet_immediately(recv_buf)?, timestamp));
        }
        // trace!("Sensor did not wake for read");
        Ok((0, SystemTime::now()))
    }
}
