// Copyright 2025 Au-Zone Technologies Inc.
// SPDX-License-Identifier: Apache-2.0

// extern crate gpiod;
use ::std::ops::Not;
use gpiod::{Chip, Input, Lines, Options, Output};
use std::io;
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
}

pub struct GpiodOut {
    output: Lines<Output>,
}
impl GpiodOut {
    pub fn new(chip: &Chip, pin: u32) -> io::Result<GpiodOut> {
        let opts = Options::output([pin]) // configure lines offsets
            .values([false]) // optionally set initial values
            .consumer("imu-driver"); // optionally set consumer string

        Ok(GpiodOut {
            output: chip.request_lines(opts)?,
        })
    }
}

impl OutputPin for GpiodOut {
    type Error = io::Error;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.output.set_values([false])?;
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.output.set_values([true])?;
        Ok(())
    }
}

pub struct GpiodIn {
    input: Lines<Input>,
}
impl GpiodIn {
    pub fn new(chip: &Chip, pin: u32) -> io::Result<GpiodIn> {
        let opts = Options::input([pin]) // configure lines offsets
            .consumer("imu-driver"); // optionally set consumer string

        Ok(GpiodIn {
            input: chip.request_lines(opts)?,
        })
    }
}

impl InputPin for GpiodIn {
    type Error = io::Error;

    /// Is the input pin high?
    fn is_high(&self) -> Result<bool, Self::Error> {
        let values = self.input.get_values([false])?;
        Ok(values[0])
    }

    /// Is the input pin low?
    fn is_low(&self) -> Result<bool, Self::Error> {
        let values = self.input.get_values([false])?;
        Ok(!values[0])
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
