//! Delays
//!
//! # What's the difference between these traits and the `timer::CountDown` trait?
//!
//! The `Timer` trait provides a *non-blocking* timer abstraction and it's meant to be used to build
//! higher level abstractions like I/O operations with timeouts. OTOH, these delays traits only
//! provide *blocking* functionality. Note that you can also use the `timer::CountDown` trait to
//! implement blocking delays.

/// Millisecond delay
///
/// `UXX` denotes the range type of the delay time. `UXX` can be `u8`, `u16`, etc. A single type can
/// implement this trait for different types of `UXX`.
use std::{thread, time::Duration};
pub trait DelayMs {
    /// Pauses execution for `ms` milliseconds
    fn delay_ms(&mut self, ms: u8);
}

pub struct TimerMs {}

impl DelayMs for TimerMs {
    fn delay_ms(&mut self, ms: u8) {
        let time = Duration::from_millis(ms.into());
        thread::sleep(time);
    }
}
