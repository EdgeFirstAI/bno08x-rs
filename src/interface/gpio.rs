// extern crate gpiod;
use ::std::ops::Not;
use gpiod::{
    AsValuesMut, Chip, Direction, EdgeDetect, Input, Lines, Masked, Options,
    Output,
};
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
    /// *NOTE* the actual electrical state of the pin may not actually be low, e.g. due to external
    /// electrical sources
    fn set_low(&mut self) -> Result<(), Self::Error>;

    /// Drives the pin high
    ///
    /// *NOTE* the actual electrical state of the pin may not actually be high, e.g. due to external
    /// electrical sources
    fn set_high(&mut self) -> Result<(), Self::Error>;

    /// Drives the pin high or low depending on the provided value
    ///
    /// *NOTE* the actual electrical state of the pin may not actually be high or low, e.g. due to external
    /// electrical sources
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
            .consumer("my-outputs"); // optionally set consumer string

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
            .consumer("my-outputs"); // optionally set consumer string

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
        Ok(values[0] == true)
    }

    /// Is the input pin low?
    fn is_low(&self) -> Result<bool, Self::Error> {
        let values = self.input.get_values([false])?;
        Ok(values[0] == false)
    }
}
