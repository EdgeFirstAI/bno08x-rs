// Copyright 2025 Au-Zone Technologies Inc.
// SPDX-License-Identifier: Apache-2.0

extern crate spidev;

use log::trace;
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};

use crate::constants::PACKET_RECV_BUF_LEN;
use std::{io, path::Path};

/// Default SPI clock. The BNO08x supports up to 3 MHz; 1 MHz keeps a full
/// 2 KiB packet under 20 ms and a typical 20-byte report under 0.2 ms, so
/// the sensor's H_INTN is serviced well inside its timing budget while still
/// leaving margin on marginal wiring.
pub const DEFAULT_SPI_SPEED_HZ: u32 = 1_000_000;

/// Blocking transfer
pub trait Transfer {
    /// Error type
    type Error;

    /// Sends `words` to the slave. Returns the `words` received from the slave
    fn transfer<'a>(&'a mut self, words: &'a mut [u8]) -> Result<&'a [u8], Self::Error>;
}

/// Blocking write
pub trait Write {
    /// Error type
    type Error;

    /// Sends `words` to the slave, ignoring all the incoming words
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error>;
}

pub struct SpiDevice {
    spi: Spidev,
    /// Scratch receive buffer so the hot read path does not allocate.
    rx_buf: [u8; PACKET_RECV_BUF_LEN],
}
impl SpiDevice {
    /// Open `path` at [`DEFAULT_SPI_SPEED_HZ`].
    pub fn new<P: AsRef<Path>>(path: P) -> io::Result<SpiDevice> {
        Self::new_with_speed(path, DEFAULT_SPI_SPEED_HZ)
    }

    /// Open `path` with an explicit SPI clock in Hz (SPI mode 3, MSB first).
    pub fn new_with_speed<P: AsRef<Path>>(path: P, max_speed_hz: u32) -> io::Result<SpiDevice> {
        let mut spi = Spidev::open(path)?;
        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(max_speed_hz)
            .mode(SpiModeFlags::SPI_MODE_3)
            .lsb_first(false)
            .build();
        spi.configure(&options)?;

        Ok(SpiDevice {
            spi,
            rx_buf: [0; PACKET_RECV_BUF_LEN],
        })
    }

    fn check_len(&self, len: usize, what: &str) -> io::Result<()> {
        if len > self.rx_buf.len() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "{} of {} bytes exceeds buffer of {}",
                    what,
                    len,
                    self.rx_buf.len()
                ),
            ));
        }
        Ok(())
    }
}

impl Transfer for SpiDevice {
    type Error = io::Error;

    fn transfer<'a>(&'a mut self, words: &'a mut [u8]) -> Result<&'a [u8], Self::Error> {
        self.check_len(words.len(), "transfer")?;
        let buf = &mut self.rx_buf[..words.len()];
        trace!("Transfer write: {:?}", words);
        let mut transfer = SpidevTransfer::read_write(words, buf);
        self.spi.transfer(&mut transfer)?;
        words.clone_from_slice(buf);
        trace!("Transfer read: {:?}", words);
        Ok(words)
    }
}

impl Write for SpiDevice {
    type Error = io::Error;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.check_len(words.len(), "write")?;
        trace!("Write: {:?}", words);
        let buf = &mut self.rx_buf[..words.len()];
        let mut transfer = SpidevTransfer::read_write(words, buf);
        self.spi.transfer(&mut transfer)?;
        trace!("Write read: {:?}", buf);
        Ok(())
    }
}
