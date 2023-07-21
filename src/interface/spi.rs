use super::SensorInterface;
use crate::interface::delay::DelayMs;
use crate::interface::gpio::{InputPin, OutputPin};
use crate::interface::spidev::{SpiDevice, Transfer, Write};
use crate::interface::{SensorCommon, PACKET_HEADER_LENGTH};
use crate::Error;
use crate::Error::SensorUnresponsive;

//#[cfg(feature = "rttdebug")]
// use panic_rtt_core::println;
use ::std::println;

/// Encapsulates all the lines required to operate this sensor
/// - SCK: clock line from master
/// - MISO: Data input from the sensor to the master
/// - MOSI: Output from the master to the sensor
/// - CSN: chip select line that selects the device on the shared SPI bus
/// - HINTN: Hardware Interrupt. Sensor uses this to indicate it had data available for read
/// - RSTN: Reset the device
pub struct SpiControlLines<SPI, /* CSN, */ IN, RSTN> {
    pub spi: SPI, // the spidev read/write
    // pub csn: CSN,    // chip select pin, SPI_CS
    pub hintn: IN,   // interrupt, IMU_INT
    pub reset: RSTN, // reset, IMU_RST
}

/// This combines the SPI peripheral and associated control pins
///
pub struct SpiInterface<SPI, /* CSN, */ IN, RSTN> {
    spi: SPI,
    // csn: CSN,
    hintn: IN,
    reset: RSTN,
    received_packet_count: usize,
}

impl<SPI, /* CSN,  */ IN, RSTN, CommE, PinE>
    SpiInterface<SPI, /* CSN,  */ IN, RSTN>
where
    SPI: Write<Error = CommE> + Transfer<Error = CommE>,
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

    /// Wait for sensor to be ready.
    /// After reset this can take around 120 ms
    /// Return true if the sensor is awake, false if it doesn't wake up
    /// `max_ms` maximum milliseconds to await for HINTN change
    fn wait_for_sensor_awake(
        &mut self,
        delay_source: &mut impl DelayMs,
        max_ms: u8,
    ) -> bool {
        for _ in 0..max_ms {
            if self.hintn_signaled() {
                return true;
            }
            delay_source.delay_ms(1);
        }

        false
    }

    /// block on HINTN for n cycles
    fn block_on_hintn(&mut self, max_cycles: usize) -> bool {
        for _ in 0..max_cycles {
            if self.hintn_signaled() {
                return true;
            }
        }
        //#[cfg(feature = "rttdebug")]
        println!("no hintn??");

        false
    }
}

impl<SPI, /* CSN, */ IN, RS, CommE, PinE> SensorInterface
    for SpiInterface<SPI, /* CSN, */ IN, RS>
where
    SPI: Write<Error = CommE> + Transfer<Error = CommE>,
    // CSN: OutputPin<Error = PinE>,
    IN: InputPin<Error = PinE>,
    RS: OutputPin<Error = PinE>,
    CommE: core::fmt::Debug,
    PinE: core::fmt::Debug,
{
    type SensorError = Error<CommE, PinE>;

    fn requires_soft_reset(&self) -> bool {
        false
    }

    fn setup(
        &mut self,
        delay_source: &mut impl DelayMs,
    ) -> Result<(), Self::SensorError> {
        // Deselect sensor
        // self.csn.set_high().map_err(Error::Pin)?;
        // Note: This assumes that WAK/PS0 is set to high already
        //TODO allow the user to provide a WAK pin
        // should already be high by default, but just in case...
        self.reset.set_high().map_err(Error::Pin)?;

        // //#[cfg(feature = "rttdebug")]
        println!("reset cycle... ");
        // reset cycle

        self.reset.set_low().map_err(Error::Pin)?;
        delay_source.delay_ms(2);
        self.reset.set_high().map_err(Error::Pin)?;

        // wait for sensor to set hintn pin after reset
        let ready = self.wait_for_sensor_awake(delay_source, 200u8);
        if !ready {
            //#[cfg(feature = "rttdebug")]
            println!("sensor not ready");
            return Err(SensorUnresponsive);
        }

        Ok(())
    }

    fn send_and_receive_packet(
        &mut self,
        send_buf: &[u8],
        recv_buf: &mut [u8],
    ) -> Result<usize, Self::SensorError> {
        //zero the receive buffer
        for i in recv_buf[..].iter_mut() {
            *i = 0;
        }

        let mut tmp = &mut [0u8; PACKET_HEADER_LENGTH];
        // check how long the message to read is
        let mut read_packet_len = 0;
        let rc = self.spi.transfer(&mut tmp[..]);
        if !rc.is_err() {
            read_packet_len =
                SensorCommon::parse_packet_header(&tmp[..PACKET_HEADER_LENGTH]);
        }

        // Copy the write message into the buffer
        for i in 0..send_buf.len() {
            recv_buf[i] = send_buf[i];
        }
        let total_packet_len = std::cmp::max(read_packet_len, send_buf.len());
        if (total_packet_len > recv_buf.len()) {
            // TODO: throw Err()
            eprintln!("Total packet length greater than recv buffer size");
        }
        let rc = self.spi.transfer(&mut recv_buf[..total_packet_len]);
        if !rc.is_err() {
            read_packet_len = SensorCommon::parse_packet_header(
                &recv_buf[..PACKET_HEADER_LENGTH],
            );
        }
        // println!("recv_buf: {:?}", &recv_buf[..read_packet_len]);

        if read_packet_len > 0 {
            self.received_packet_count += 1;
        }
        Ok(read_packet_len)
        // self.write_packet(send_buf)?;

        // if !self.block_on_hintn(1000) {
        //     //no packet to be read
        //     //#[cfg(feature = "rttdebug")]
        //     println!("no packet to read?");
        //     return Ok(0);
        // }
        // self.read_packet(recv_buf)
    }

    fn write_packet(&mut self, packet: &[u8]) -> Result<(), Self::SensorError> {
        // self.csn.set_low().map_err(Error::Pin)?;
        let rc = self.spi.write(&packet).map_err(Error::Comm);
        // self.csn.set_high().map_err(Error::Pin)?;
        if rc.is_err() {
            return Err(rc.unwrap_err());
        }

        Ok(())
    }

    /// Read a complete packet from the sensor
    fn read_packet(
        &mut self,
        recv_buf: &mut [u8],
    ) -> Result<usize, Self::SensorError> {
        // Note: HINTN cannot always be used to detect data ready.
        // As soon as host selects CSN, HINTN resets

        // check how long the message to read is
        let mut read_packet_len = 0;
        for i in recv_buf[..PACKET_HEADER_LENGTH].iter_mut() {
            *i = 0;
        }
        let rc = self.spi.transfer(&mut recv_buf[..PACKET_HEADER_LENGTH]);
        if !rc.is_err() {
            read_packet_len = SensorCommon::parse_packet_header(
                &recv_buf[..PACKET_HEADER_LENGTH],
            );
        }

        //zero the receive buffer
        for i in recv_buf[..read_packet_len].iter_mut() {
            *i = 0;
        }
        let rc = self.spi.transfer(&mut recv_buf[..read_packet_len]);
        if !rc.is_err() {
            read_packet_len = SensorCommon::parse_packet_header(
                &recv_buf[..PACKET_HEADER_LENGTH],
            );
        }
        // println!("recv_buf: {:?}", &recv_buf[..read_packet_len]);

        if read_packet_len > 0 {
            self.received_packet_count += 1;
        }

        Ok(read_packet_len)
    }

    fn read_with_timeout(
        &mut self,
        recv_buf: &mut [u8],
        delay_source: &mut impl DelayMs,
        max_ms: u8,
    ) -> Result<usize, Self::SensorError> {
        if self.wait_for_sensor_awake(delay_source, max_ms) {
            return self.read_packet(recv_buf);
        }
        // println!("Sensor did not wake for read");
        Ok(0)
    }
}
