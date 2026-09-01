// Copyright 2025 Au-Zone Technologies Inc.
// SPDX-License-Identifier: Apache-2.0

//! SPI transport for the BNO08x.
//!
//! # Timing model
//!
//! The BNO08x is an SPI slave that only participates in a transaction while
//! it is awake and has signalled the host via H_INTN (active low). Two rules
//! from the CEVA datasheet drive the design of this module:
//!
//! 1. After reset the sensor hub sleeps until a feature is enabled, and in SPI
//!    mode the host can only wake it via the PS0/WAKE pin. On boards that strap
//!    PS0 high there is no WAKE, so a write is only guaranteed to be accepted
//!    while H_INTN is asserted (the sensor is awake and has a packet pending).
//!    Every write here is therefore gated on H_INTN.
//! 2. On the BNO085/BNO086 the host must service H_INTN within roughly 1/10 of
//!    the fastest report period, otherwise the hub times out, retries and
//!    starves its own processing. Packet reads therefore wait on the H_INTN
//!    edge rather than sleeping for a fixed interval.

use log::{debug, error, trace};

use super::SensorInterface;
use crate::{
    interface::{
        gpio::{InputPin, OutputPin},
        spidev::{Transfer, Write},
        SensorCommon, PACKET_HEADER_LENGTH,
    },
    Error,
    Error::{BufferOverflow, NoDataAvailable, SensorUnresponsive},
};
use std::{
    fmt::Debug,
    thread::sleep,
    time::{Duration, Instant},
};

/// Interval between H_INTN polls. Short enough that a 5 ms rotation vector
/// period is serviced well inside the datasheet's 1/10 period guidance.
const HINTN_POLL_INTERVAL: Duration = Duration::from_micros(100);

/// How long to wait after a hard reset for the sensor to start driving
/// H_INTN high (datasheet t1 internal initialisation is at least 90 ms).
const BOOT_TIMEOUT: Duration = Duration::from_millis(150);

/// How long the sensor may take to assert H_INTN after a hard reset.
const RESET_TIMEOUT: Duration = Duration::from_millis(500);

/// How long to wait for the sensor to re-assert H_INTN for the remainder of a
/// packet after the header-only read deasserted chip select. The datasheet
/// specifies re-assertion in microseconds; this is generous.
const CONTINUATION_TIMEOUT: Duration = Duration::from_millis(5);

/// How long a write may wait for the sensor to become ready. Right after
/// reset the startup burst arrives within a few hundred milliseconds; once a
/// report is enabled H_INTN asserts every report period.
const WRITE_READY_TIMEOUT: Duration = Duration::from_millis(1000);

/// Width of the reset pulse driven on RSTN.
const RESET_PULSE: Duration = Duration::from_millis(2);

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

    /// Wait for H_INTN to be asserted, polling at [`HINTN_POLL_INTERVAL`].
    ///
    /// Returns true as soon as the sensor signals it needs attention, false
    /// if `timeout` elapses first. Always samples the line at least once so
    /// a zero timeout acts as a non-blocking check.
    fn wait_for_hintn(&self, timeout: Duration) -> bool {
        let deadline = Instant::now() + timeout;
        loop {
            if self.hintn_signaled() {
                return true;
            }
            if Instant::now() >= deadline {
                return false;
            }
            sleep(HINTN_POLL_INTERVAL);
        }
    }

    /// Wait for H_INTN to be deasserted (driven high by a running sensor).
    fn wait_for_hintn_deasserted(&self, timeout: Duration) -> bool {
        let deadline = Instant::now() + timeout;
        loop {
            if !self.hintn_signaled() {
                return true;
            }
            if Instant::now() >= deadline {
                return false;
            }
            sleep(HINTN_POLL_INTERVAL);
        }
    }

    /// Read the 4-byte SHTP header of the pending packet and return the
    /// total packet length it announces (0 for an idle or garbage header).
    fn read_header(&mut self, recv_buf: &mut [u8]) -> Result<usize, Error<CommE, PinE>> {
        recv_buf[..PACKET_HEADER_LENGTH].fill(0);
        self.spi
            .transfer(&mut recv_buf[..PACKET_HEADER_LENGTH])
            .map_err(Error::Comm)?;
        Ok(SensorCommon::parse_packet_header(
            &recv_buf[..PACKET_HEADER_LENGTH],
        ))
    }

    /// After a header-only read has deasserted chip select the sensor
    /// re-asserts H_INTN to offer the rest of the packet as a continuation.
    /// Wait for that so the next transaction starts on a ready sensor.
    fn wait_for_continuation(&self) {
        if !self.wait_for_hintn(CONTINUATION_TIMEOUT) {
            debug!("H_INTN not re-asserted for packet continuation, reading anyway");
        }
    }
}

impl<SPI, /* CSN, */ IN, RS, CommE, PinE> SensorInterface
    for SpiInterface<SPI, /* CSN, */ IN, RS>
where
    SPI: Write<Error = CommE> + Transfer<Error = CommE>,
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
        //TODO allow the user to provide a WAK pin
        // should already be high by default, but just in case...
        self.reset.set_high().map_err(Error::Pin)?;

        trace!("reset cycle... ");
        // reset cycle

        self.reset.set_low().map_err(Error::Pin)?;
        sleep(RESET_PULSE);
        self.reset.set_high().map_err(Error::Pin)?;

        // While the sensor boots (t1, at least 90 ms) its H_INTN output is
        // not yet driven, so on a board whose pad defaults to a pull-down the
        // line reads asserted before the sensor is alive. Wait for the sensor
        // to drive H_INTN high first; on a pulled-up board this returns at
        // once.
        if !self.wait_for_hintn_deasserted(BOOT_TIMEOUT) {
            debug!("H_INTN never deasserted after reset, assuming sensor drives it low from boot");
        }

        // The sensor asserts H_INTN once its advertisement is ready. From
        // this point on the host must keep up: the startup burst is the only
        // window in which a write is guaranteed to land without a WAKE pin.
        if !self.wait_for_hintn(RESET_TIMEOUT) {
            return Err(SensorUnresponsive);
        }

        Ok(())
    }

    /// Send a packet while reading whatever the sensor has pending.
    ///
    /// The write is gated on H_INTN: without a WAKE pin a write to a sleeping
    /// sensor is silently discarded, so if the sensor does not signal
    /// readiness within [`WRITE_READY_TIMEOUT`] this returns
    /// [`NoDataAvailable`] rather than pretending the command was delivered.
    fn send_and_receive_packet(
        &mut self,
        send_buf: &[u8],
        recv_buf: &mut [u8],
    ) -> Result<usize, Self::SensorError> {
        // The outgoing packet is copied into the receive buffer to be clocked
        // out, so it must fit before anything is written there.
        if send_buf.len() > recv_buf.len() {
            error!(
                "Send buffer length ({}) greater than receive buffer size ({})",
                send_buf.len(),
                recv_buf.len()
            );
            return Err(BufferOverflow {
                packet_size: send_buf.len(),
                buffer_size: recv_buf.len(),
            });
        }

        if !self.wait_for_hintn(WRITE_READY_TIMEOUT) {
            error!("Sensor not ready (H_INTN idle) - write would be lost");
            return Err(NoDataAvailable);
        }

        // Learn how long the pending packet is so the combined transfer
        // clocks enough bytes to receive all of it.
        let pending_len = self.read_header(recv_buf)?;

        let total_packet_len = std::cmp::max(pending_len, send_buf.len());
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

        recv_buf.fill(0);
        recv_buf[..send_buf.len()].copy_from_slice(send_buf);

        if pending_len > 0 {
            self.wait_for_continuation();
        }
        self.spi
            .transfer(&mut recv_buf[..total_packet_len])
            .map_err(Error::Comm)?;

        let read_packet_len = SensorCommon::parse_packet_header(&recv_buf[..PACKET_HEADER_LENGTH]);
        if read_packet_len > 0 {
            self.received_packet_count += 1;
        }
        Ok(read_packet_len)
    }

    fn write_packet(&mut self, packet: &[u8]) -> Result<(), Self::SensorError> {
        if !self.wait_for_hintn(WRITE_READY_TIMEOUT) {
            error!("Sensor not ready (H_INTN idle) - write would be lost");
            return Err(NoDataAvailable);
        }
        self.spi.write(packet).map_err(Error::Comm)?;
        Ok(())
    }

    /// Read a complete packet from the sensor.
    ///
    /// Callers are expected to have observed H_INTN asserted (see
    /// [`read_with_timeout`](Self::read_with_timeout)); this performs the
    /// header read, waits for the continuation, then reads the body.
    fn read_packet(&mut self, recv_buf: &mut [u8]) -> Result<usize, Self::SensorError> {
        if !self.wait_for_hintn(Duration::ZERO) {
            error!("No message to read - H_INTN not asserted");
            return Err(NoDataAvailable);
        }
        // As soon as host selects CSN, HINTN resets

        let read_packet_len = self.read_header(recv_buf)?;
        if read_packet_len <= PACKET_HEADER_LENGTH {
            // Idle sensor (zero packet) or header-only packet: nothing to
            // read and nothing worth handing to the protocol layer.
            return Ok(0);
        }

        // The sensor re-sends the header (with the continuation bit set and
        // the length restated) followed by the cargo, so the whole announced
        // length is transferred here.
        recv_buf[..read_packet_len].fill(0);
        self.wait_for_continuation();
        self.spi
            .transfer(&mut recv_buf[..read_packet_len])
            .map_err(Error::Comm)?;

        let read_packet_len = SensorCommon::parse_packet_header(&recv_buf[..PACKET_HEADER_LENGTH]);
        if read_packet_len > 0 {
            self.received_packet_count += 1;
        }

        Ok(read_packet_len)
    }

    fn read_with_timeout(
        &mut self,
        recv_buf: &mut [u8],
        max_ms: usize,
    ) -> Result<usize, Self::SensorError> {
        if self.wait_for_hintn(Duration::from_millis(max_ms as u64)) {
            return self.read_packet(recv_buf);
        }
        Ok(0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{
        atomic::{AtomicBool, AtomicUsize, Ordering},
        Arc,
    };

    #[derive(Debug, PartialEq)]
    struct MockCommError;

    #[derive(Debug, PartialEq)]
    struct MockPinError;

    /// SPI stub that replays canned responses and records what was clocked out.
    #[derive(Default)]
    struct MockSpi {
        /// Bytes returned by successive `transfer` calls.
        responses: Vec<Vec<u8>>,
        /// What each `transfer`/`write` call sent.
        sent: Vec<Vec<u8>>,
        fail: bool,
    }

    impl MockSpi {
        fn with_responses(responses: Vec<Vec<u8>>) -> Self {
            Self {
                responses,
                ..Default::default()
            }
        }

        fn failing() -> Self {
            Self {
                fail: true,
                ..Default::default()
            }
        }
    }

    impl Transfer for MockSpi {
        type Error = MockCommError;

        fn transfer<'a>(&'a mut self, words: &'a mut [u8]) -> Result<&'a [u8], Self::Error> {
            if self.fail {
                return Err(MockCommError);
            }
            self.sent.push(words.to_vec());
            if !self.responses.is_empty() {
                let response = self.responses.remove(0);
                let n = response.len().min(words.len());
                words[..n].copy_from_slice(&response[..n]);
            }
            Ok(words)
        }
    }

    impl Write for MockSpi {
        type Error = MockCommError;

        fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            if self.fail {
                return Err(MockCommError);
            }
            self.sent.push(words.to_vec());
            Ok(())
        }
    }

    /// Interrupt line whose level the test controls. Active low, so
    /// `asserted == true` means the sensor wants attention.
    #[derive(Clone)]
    struct MockHintn {
        asserted: Arc<AtomicBool>,
        /// Flip to the opposite level after this many reads (0 = never).
        flip_after: Arc<AtomicUsize>,
        reads: Arc<AtomicUsize>,
    }

    impl MockHintn {
        fn new(asserted: bool) -> Self {
            Self {
                asserted: Arc::new(AtomicBool::new(asserted)),
                flip_after: Arc::new(AtomicUsize::new(0)),
                reads: Arc::new(AtomicUsize::new(0)),
            }
        }

        /// Start at `asserted` and flip once `after` reads have happened.
        fn flipping(asserted: bool, after: usize) -> Self {
            let pin = Self::new(asserted);
            pin.flip_after.store(after, Ordering::SeqCst);
            pin
        }
    }

    impl InputPin for MockHintn {
        type Error = MockPinError;

        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(!self.is_low()?)
        }

        fn is_low(&self) -> Result<bool, Self::Error> {
            let n = self.reads.fetch_add(1, Ordering::SeqCst) + 1;
            let flip_after = self.flip_after.load(Ordering::SeqCst);
            if flip_after > 0 && n == flip_after {
                let current = self.asserted.load(Ordering::SeqCst);
                self.asserted.store(!current, Ordering::SeqCst);
            }
            Ok(self.asserted.load(Ordering::SeqCst))
        }
    }

    #[derive(Clone, Default)]
    struct MockReset {
        transitions: Arc<std::sync::Mutex<Vec<bool>>>,
    }

    impl OutputPin for MockReset {
        type Error = MockPinError;

        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.transitions.lock().unwrap().push(false);
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.transitions.lock().unwrap().push(true);
            Ok(())
        }
    }

    fn interface(
        spi: MockSpi,
        hintn: MockHintn,
        reset: MockReset,
    ) -> SpiInterface<MockSpi, MockHintn, MockReset> {
        SpiInterface::new(SpiControlLines { spi, hintn, reset })
    }

    /// A valid 20-byte SHTP packet on the hub control channel.
    fn packet(len: usize) -> Vec<u8> {
        let mut p = vec![0u8; len];
        p[0] = len as u8;
        p[2] = 2;
        p
    }

    #[test]
    fn spi_never_requires_soft_reset() {
        let iface = interface(
            MockSpi::default(),
            MockHintn::new(true),
            MockReset::default(),
        );
        assert!(!iface.requires_soft_reset());
    }

    #[test]
    fn setup_pulses_reset_and_waits_for_sensor() {
        // High while booting, then asserted: the sequence a live sensor gives.
        let hintn = MockHintn::flipping(false, 2);
        let reset = MockReset::default();
        let mut iface = interface(MockSpi::default(), hintn, reset.clone());

        assert!(iface.setup().is_ok());
        // High to deassert, low for the pulse, high again to release.
        assert_eq!(*reset.transitions.lock().unwrap(), vec![true, false, true]);
    }

    #[test]
    fn setup_reports_unresponsive_when_hintn_never_asserts() {
        // Sensor drives the line high and never asks for attention.
        let mut iface = interface(
            MockSpi::default(),
            MockHintn::new(false),
            MockReset::default(),
        );
        assert!(matches!(iface.setup(), Err(SensorUnresponsive)));
    }

    #[test]
    fn write_is_rejected_when_sensor_is_idle() {
        // A sleeping hub silently discards writes, so the interface must
        // refuse rather than report success.
        let mut iface = interface(
            MockSpi::default(),
            MockHintn::new(false),
            MockReset::default(),
        );
        let mut recv = vec![0u8; 64];
        assert!(matches!(
            iface.send_and_receive_packet(&[1, 2, 3, 4], &mut recv),
            Err(NoDataAvailable)
        ));
        assert!(matches!(
            iface.write_packet(&[1, 2, 3, 4]),
            Err(NoDataAvailable)
        ));
    }

    #[test]
    fn write_packet_reaches_the_bus_when_sensor_is_ready() {
        let mut iface = interface(
            MockSpi::default(),
            MockHintn::new(true),
            MockReset::default(),
        );
        assert!(iface.write_packet(&[9, 8, 7]).is_ok());
        assert_eq!(iface.spi.sent, vec![vec![9, 8, 7]]);
    }

    #[test]
    fn send_and_receive_rejects_send_buffer_larger_than_receive_buffer() {
        let mut iface = interface(
            MockSpi::default(),
            MockHintn::new(true),
            MockReset::default(),
        );
        let mut recv = vec![0u8; 4];
        assert!(matches!(
            iface.send_and_receive_packet(&[0; 16], &mut recv),
            Err(BufferOverflow {
                packet_size: 16,
                buffer_size: 4
            })
        ));
    }

    #[test]
    fn send_and_receive_rejects_pending_packet_larger_than_receive_buffer() {
        // Header announces 200 bytes but the caller offered 64.
        let mut header = vec![0u8; 8];
        header[0] = 200;
        let mut iface = interface(
            MockSpi::with_responses(vec![header]),
            MockHintn::new(true),
            MockReset::default(),
        );
        let mut recv = vec![0u8; 64];
        assert!(matches!(
            iface.send_and_receive_packet(&[1, 2, 3, 4], &mut recv),
            Err(BufferOverflow {
                packet_size: 200,
                buffer_size: 64
            })
        ));
    }

    #[test]
    fn send_and_receive_clocks_out_the_send_buffer() {
        let mut iface = interface(
            MockSpi::with_responses(vec![packet(20), packet(20)]),
            MockHintn::new(true),
            MockReset::default(),
        );
        let mut recv = vec![0u8; 64];
        let len = iface
            .send_and_receive_packet(&[6, 0, 2, 0, 0xF9, 0], &mut recv)
            .expect("send succeeds");
        assert_eq!(len, 20);
        // Second transfer carries the caller's packet, padded to the pending
        // packet length so the whole response is clocked in.
        let second = &iface.spi.sent[1];
        assert_eq!(&second[..6], &[6, 0, 2, 0, 0xF9, 0]);
        assert_eq!(second.len(), 20);
    }

    #[test]
    fn send_and_receive_propagates_comm_errors() {
        let mut iface = interface(
            MockSpi::failing(),
            MockHintn::new(true),
            MockReset::default(),
        );
        let mut recv = vec![0u8; 64];
        assert!(matches!(
            iface.send_and_receive_packet(&[1, 2, 3, 4], &mut recv),
            Err(Error::Comm(MockCommError))
        ));
    }

    #[test]
    fn read_packet_requires_an_asserted_interrupt() {
        let mut iface = interface(
            MockSpi::default(),
            MockHintn::new(false),
            MockReset::default(),
        );
        let mut recv = vec![0u8; 64];
        assert!(matches!(iface.read_packet(&mut recv), Err(NoDataAvailable)));
    }

    #[test]
    fn read_packet_ignores_an_idle_sensor() {
        // An awake but idle hub answers with a zero-length header.
        let mut iface = interface(
            MockSpi::with_responses(vec![vec![0, 0, 0, 0]]),
            MockHintn::new(true),
            MockReset::default(),
        );
        let mut recv = vec![0u8; 64];
        assert_eq!(iface.read_packet(&mut recv).expect("no error"), 0);
    }

    #[test]
    fn read_packet_ignores_a_header_only_packet() {
        let mut iface = interface(
            MockSpi::with_responses(vec![vec![4, 0, 2, 0]]),
            MockHintn::new(true),
            MockReset::default(),
        );
        let mut recv = vec![0u8; 64];
        assert_eq!(iface.read_packet(&mut recv).expect("no error"), 0);
    }

    #[test]
    fn read_packet_returns_the_full_packet() {
        let mut iface = interface(
            MockSpi::with_responses(vec![packet(20), packet(20)]),
            MockHintn::new(true),
            MockReset::default(),
        );
        let mut recv = vec![0u8; 64];
        assert_eq!(iface.read_packet(&mut recv).expect("no error"), 20);
        assert_eq!(recv[0], 20);
    }

    #[test]
    fn read_with_timeout_returns_zero_when_nothing_arrives() {
        let mut iface = interface(
            MockSpi::default(),
            MockHintn::new(false),
            MockReset::default(),
        );
        let mut recv = vec![0u8; 64];
        assert_eq!(iface.read_with_timeout(&mut recv, 1).expect("no error"), 0);
    }

    #[test]
    fn read_with_timeout_reads_once_the_sensor_signals() {
        let mut iface = interface(
            MockSpi::with_responses(vec![packet(20), packet(20)]),
            MockHintn::new(true),
            MockReset::default(),
        );
        let mut recv = vec![0u8; 64];
        assert_eq!(
            iface.read_with_timeout(&mut recv, 10).expect("no error"),
            20
        );
    }
}
