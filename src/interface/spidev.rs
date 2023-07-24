extern crate spidev;
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::convert::TryInto;
use std::io::Read as StdRead;
use std::io::Write as StdWrite;
use std::path::Path;
use std::println;
use std::{io, vec};
/// Blocking transfer
pub trait Transfer {
    /// Error type
    type Error;

    /// Sends `words` to the slave. Returns the `words` received from the slave
    fn transfer<'a>(
        &'a mut self,
        words: &'a mut [u8],
    ) -> Result<&[u8], Self::Error>;
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
}
impl SpiDevice {
    pub fn new<P: AsRef<Path>>(path: P) -> io::Result<SpiDevice> {
        let mut spi = Spidev::open(path)?;
        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(25_000)
            .mode(SpiModeFlags::SPI_MODE_3)
            .lsb_first(false)
            .build();
        spi.configure(&options)?;

        Ok(SpiDevice { spi })
    }
}

impl Transfer for SpiDevice {
    type Error = io::Error;
    fn transfer<'a>(
        &'a mut self,
        words: &'a mut [u8],
    ) -> Result<&[u8], Self::Error> {
        // println!("Transfer write: {:?}", words);
        let mut rx_buf = vec![0_u8; words.len()];
        let mut buf = rx_buf.as_mut();
        let mut transfer = SpidevTransfer::read_write(words, buf);
        self.spi.transfer(&mut transfer)?;
        // // self.spi.write(words)?;
        // // self.spi.read(&mut buf)?;

        // self.spi.read(&mut buf)?;
        words.clone_from_slice(buf);
        // println!("Transfer read: {:?}", words);
        Ok(words)
    }
}

impl Write for SpiDevice {
    type Error = io::Error;
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut rx_buf = vec![0_u8; words.len()];
        let mut buf = rx_buf.as_mut();
        let mut transfer = SpidevTransfer::read_write(words, buf);
        self.spi.transfer(&mut transfer)?;
        // self.spi.write(words)?;
        // self.spi.read(&mut buf)?;
        // println!("Write read: {:?}", buf);

        // self.spi.write(words)?;
        // self.spi.read(&mut buf)?;
        Ok(())
    }
}
