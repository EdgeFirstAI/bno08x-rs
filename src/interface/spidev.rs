// Copyright 2025 Au-Zone Technologies Inc.
// SPDX-License-Identifier: Apache-2.0

extern crate spidev;

use log::trace;
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};

use std::{io, path::Path};
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
    tmp: Vec<u8>,
}
impl SpiDevice {
    pub fn new<P: AsRef<Path>>(path: P) -> io::Result<SpiDevice> {
        let mut spi = Spidev::open(path)?;
        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(3_000_000) // max spi frequency for BNO085 is 3 MHz
            .mode(SpiModeFlags::SPI_MODE_3)
            .lsb_first(false)
            .build();
        spi.configure(&options)?;

        let tmp = Vec::with_capacity(2048);
        Ok(SpiDevice { spi, tmp })
    }
}

impl Transfer for SpiDevice {
    type Error = io::Error;

    fn transfer<'a>(&'a mut self, words: &'a mut [u8]) -> Result<&'a [u8], Self::Error> {
        self.tmp.resize(words.len(), 0);
        trace!("Transfer write: {:?}", words);
        let mut transfer = SpidevTransfer::read_write(words, &mut self.tmp);
        self.spi.transfer(&mut transfer)?;
        words.clone_from_slice(&self.tmp);
        trace!("Transfer read: {:?}", words);
        Ok(words)
    }
}

impl Write for SpiDevice {
    type Error = io::Error;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        trace!("Write: {:?}", words);
        self.tmp.resize(words.len(), 0);
        let mut transfer = SpidevTransfer::read_write(words, &mut self.tmp);
        self.spi.transfer(&mut transfer)?;
        trace!("Write read: {:?}", &self.tmp);
        Ok(())
    }
}
