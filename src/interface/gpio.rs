// Copyright 2025 Au-Zone Technologies Inc.
// SPDX-License-Identifier: Apache-2.0

use ::std::ops::Not;
use gpiocdev::{
    line::{EdgeDetection, EdgeEvent, EdgeKind, Value},
    Request,
};
use std::{sync::Arc, thread::JoinHandle};

use crate::watch_channel;
pub enum PinState {
    /// Low pin state
    Low,
    /// High pin state
    High,
}

impl From<bool> for PinState {
    fn from(value: bool) -> Self {
        match value {
            false => PinState::Low,
            true => PinState::High,
        }
    }
}

impl Not for PinState {
    type Output = PinState;

    fn not(self) -> Self::Output {
        match self {
            PinState::High => PinState::Low,
            PinState::Low => PinState::High,
        }
    }
}

pub trait OutputPin {
    /// Error type
    type Error;

    /// Drives the pin low
    ///
    /// *NOTE* the actual electrical state of the pin may not actually be low,
    /// e.g. due to external electrical sources
    fn set_low(&mut self) -> Result<(), Self::Error>;

    /// Drives the pin high
    ///
    /// *NOTE* the actual electrical state of the pin may not actually be high,
    /// e.g. due to external electrical sources
    fn set_high(&mut self) -> Result<(), Self::Error>;

    /// Drives the pin high or low depending on the provided value
    ///
    /// *NOTE* the actual electrical state of the pin may not actually be high
    /// or low, e.g. due to external electrical sources
    fn set_state(&mut self, state: PinState) -> Result<(), Self::Error> {
        match state {
            PinState::Low => self.set_low(),
            PinState::High => self.set_high(),
        }
    }
}

pub trait InputPin {
    /// Error type
    type Error;

    /// Is the input pin high?
    fn is_high(&self) -> Result<bool, Self::Error>;

    /// Is the input pin low?
    fn is_low(&self) -> Result<bool, Self::Error>;

    /// read events
    fn read_event(&mut self) -> Result<EdgeEvent, Self::Error>;

    /// read events, returns Ok(None) on timeout
    fn read_event_with_timeout(
        &mut self,
        timeout: std::time::Duration,
    ) -> Result<Option<EdgeEvent>, Self::Error>;
}

pub struct GpiodOut {
    output: Request,
}
impl GpiodOut {
    pub fn new(chip: &str, pin: u32) -> gpiocdev::Result<GpiodOut> {
        let output = Request::builder()
            .on_chip(chip)
            .with_line(pin)
            .as_output(Value::Inactive)
            .request()?;

        Ok(GpiodOut { output })
    }
}

impl OutputPin for GpiodOut {
    type Error = gpiocdev::Error;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.output.set_lone_value(Value::Inactive)?;
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.output.set_lone_value(Value::Active)?;
        Ok(())
    }
}

struct JoinOnDrop(Option<JoinHandle<()>>);
impl Drop for JoinOnDrop {
    fn drop(&mut self) {
        if let Some(handle) = self.0.take() {
            if let Err(e) = handle.join() {
                log::error!("Error joining thread: {:?}", e);
            }
        }
    }
}

pub struct GpiodIn {
    input: Arc<Request>,
    rx: watch_channel::Receiver<EdgeEvent>,
    _tx_handle: JoinOnDrop, /* _tx_handle must be after rx to ensure that the rx is dropped
                             * before joining the thread */
}
impl GpiodIn {
    pub fn new(chip: &str, pin: u32) -> gpiocdev::Result<GpiodIn> {
        let input = Request::builder()
            .on_chip(chip)
            .with_line(pin)
            .as_active_high()
            .with_edge_detection(EdgeDetection::BothEdges)
            .request()?;
        let (tx, rx) = watch_channel::channel(EdgeEvent {
            timestamp_ns: 0,
            kind: gpiocdev::line::EdgeKind::Rising,
            offset: pin,
            seqno: 0,
            line_seqno: 0,
        });
        let input = Arc::new(input);
        let input_clone = Arc::clone(&input);
        let tx_handle = std::thread::spawn(move || {
            for e in input_clone.edge_events() {
                let Ok(mut e) = e else {
                    log::error!("Error reading edge event: {:?}", e);
                    continue;
                };
                match input_clone.lone_value().unwrap() {
                    Value::Active => e.kind = EdgeKind::Rising,
                    Value::Inactive => e.kind = EdgeKind::Falling,
                }
                if tx.send(e).is_err() {
                    break; // All receivers dropped, stop the thread
                }
            }
        });

        Ok(GpiodIn {
            input,
            rx,
            _tx_handle: JoinOnDrop(Some(tx_handle)),
        })
    }
}

impl InputPin for GpiodIn {
    type Error = gpiocdev::Error;

    /// Is the input pin high?
    fn is_high(&self) -> Result<bool, Self::Error> {
        let values = self.input.lone_value()?;
        Ok(values == Value::Active)
    }

    /// Is the input pin low?
    fn is_low(&self) -> Result<bool, Self::Error> {
        let values = self.input.lone_value()?;
        Ok(values == Value::Inactive)
    }

    /// Read events
    fn read_event(&mut self) -> Result<EdgeEvent, Self::Error> {
        self.rx
            .recv()
            .map_err(|e| gpiocdev::Error::InvalidArgument(format!("watch channel error: {:?}", e)))
    }

    /// Read events with a timeout
    fn read_event_with_timeout(
        &mut self,
        timeout: std::time::Duration,
    ) -> Result<Option<EdgeEvent>, Self::Error> {
        let curr = self.rx.borrow();
        if curr.kind == EdgeKind::Falling {
            return Ok(Some(curr));
        }
        match self.rx.recv_timeout(timeout) {
            Ok(value) => Ok(Some(value)),
            Err(_) => Ok(None), // timeout or channel closed
        }
    }
}
#[cfg(test)]
mod tests {

    use super::*;

    // ==========================================================================
    // PinState Tests
    // ==========================================================================

    #[test]
    fn test_pin_state_from_bool_false() {
        let state: PinState = false.into();
        match state {
            PinState::Low => {} // expected
            PinState::High => panic!("Expected Low, got High"),
        }
    }

    #[test]
    fn test_pin_state_from_bool_true() {
        let state: PinState = true.into();
        match state {
            PinState::High => {} // expected
            PinState::Low => panic!("Expected High, got Low"),
        }
    }

    #[test]
    fn test_pin_state_not_low() {
        let state = PinState::Low;
        let inverted = !state;
        match inverted {
            PinState::High => {} // expected
            PinState::Low => panic!("Expected High after inverting Low"),
        }
    }

    #[test]
    fn test_pin_state_not_high() {
        let state = PinState::High;
        let inverted = !state;
        match inverted {
            PinState::Low => {} // expected
            PinState::High => panic!("Expected Low after inverting High"),
        }
    }

    #[test]
    fn test_pin_state_double_inversion() {
        let original = PinState::Low;
        let double_inverted = !!original;
        match double_inverted {
            PinState::Low => {} // expected - back to original
            PinState::High => panic!("Expected Low after double inversion"),
        }

        let original = PinState::High;
        let double_inverted = !!original;
        match double_inverted {
            PinState::High => {} // expected - back to original
            PinState::Low => panic!("Expected High after double inversion"),
        }
    }

    // ==========================================================================
    // OutputPin Trait Default Implementation Tests
    // ==========================================================================

    // Mock OutputPin for testing the default set_state implementation
    struct MockOutputPin {
        state: Option<bool>,
        error_on_next: bool,
    }

    impl MockOutputPin {
        fn new() -> Self {
            Self {
                state: None,
                error_on_next: false,
            }
        }

        fn with_error() -> Self {
            Self {
                state: None,
                error_on_next: true,
            }
        }
    }

    #[derive(Debug, PartialEq)]
    struct MockPinError;

    impl OutputPin for MockOutputPin {
        type Error = MockPinError;

        fn set_low(&mut self) -> Result<(), Self::Error> {
            if self.error_on_next {
                return Err(MockPinError);
            }
            self.state = Some(false);
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            if self.error_on_next {
                return Err(MockPinError);
            }
            self.state = Some(true);
            Ok(())
        }
    }

    #[test]
    fn test_output_pin_set_state_low() {
        let mut pin = MockOutputPin::new();
        let result = pin.set_state(PinState::Low);
        assert!(result.is_ok());
        assert_eq!(pin.state, Some(false));
    }

    #[test]
    fn test_output_pin_set_state_high() {
        let mut pin = MockOutputPin::new();
        let result = pin.set_state(PinState::High);
        assert!(result.is_ok());
        assert_eq!(pin.state, Some(true));
    }

    #[test]
    fn test_output_pin_set_low_error() {
        let mut pin = MockOutputPin::with_error();
        let result = pin.set_low();
        assert_eq!(result, Err(MockPinError));
    }

    #[test]
    fn test_output_pin_set_high_error() {
        let mut pin = MockOutputPin::with_error();
        let result = pin.set_high();
        assert_eq!(result, Err(MockPinError));
    }

    #[test]
    fn test_output_pin_set_state_error_propagates() {
        let mut pin = MockOutputPin::with_error();
        let result = pin.set_state(PinState::Low);
        assert_eq!(result, Err(MockPinError));

        let mut pin = MockOutputPin::with_error();
        let result = pin.set_state(PinState::High);
        assert_eq!(result, Err(MockPinError));
    }

    // ==========================================================================
    // InputPin Trait Tests (using mock)
    // ==========================================================================

    struct MockInputPin {
        high: bool,
        error_on_read: bool,
    }

    impl MockInputPin {
        fn new(high: bool) -> Self {
            Self {
                high,
                error_on_read: false,
            }
        }

        fn with_error() -> Self {
            Self {
                high: false,
                error_on_read: true,
            }
        }
    }

    #[derive(Debug, PartialEq)]
    struct MockInputError;

    impl InputPin for MockInputPin {
        type Error = MockInputError;

        fn is_high(&self) -> Result<bool, Self::Error> {
            if self.error_on_read {
                return Err(MockInputError);
            }
            Ok(self.high)
        }

        fn is_low(&self) -> Result<bool, Self::Error> {
            if self.error_on_read {
                return Err(MockInputError);
            }
            Ok(!self.high)
        }

        fn read_event(&mut self) -> Result<EdgeEvent, Self::Error> {
            if self.error_on_read {
                return Err(MockInputError);
            }
            // Return a dummy event for testing
            Ok(EdgeEvent {
                timestamp_ns: 0,
                kind: gpiocdev::line::EdgeKind::Falling,
                offset: 0,
                seqno: 0,
                line_seqno: 0,
            })
        }

        fn read_event_with_timeout(
            &mut self,
            _timeout: std::time::Duration,
        ) -> Result<Option<EdgeEvent>, Self::Error> {
            todo!()
        }
    }

    #[test]
    fn test_input_pin_is_high_when_high() {
        let pin = MockInputPin::new(true);
        assert_eq!(pin.is_high(), Ok(true));
        assert_eq!(pin.is_low(), Ok(false));
    }

    #[test]
    fn test_input_pin_is_low_when_low() {
        let pin = MockInputPin::new(false);
        assert_eq!(pin.is_high(), Ok(false));
        assert_eq!(pin.is_low(), Ok(true));
    }

    #[test]
    fn test_input_pin_error_propagates() {
        let pin = MockInputPin::with_error();
        assert_eq!(pin.is_high(), Err(MockInputError));
        assert_eq!(pin.is_low(), Err(MockInputError));
    }

    // ==========================================================================
    // Boolean Conversion Edge Cases
    // ==========================================================================

    #[test]
    fn test_pin_state_conversion_consistency() {
        // Converting bool to PinState and checking consistency
        for &b in &[true, false] {
            let state: PinState = b.into();
            let is_high = matches!(state, PinState::High);
            assert_eq!(is_high, b, "PinState conversion should match bool value");
        }
    }

    #[test]
    fn test_pin_state_inversion_is_symmetric() {
        // !Low == High and !High == Low
        assert!(matches!(!PinState::Low, PinState::High));
        assert!(matches!(!PinState::High, PinState::Low));
    }
}
