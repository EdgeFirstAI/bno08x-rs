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

pub fn delay_ms(ms: usize) {
    let time = Duration::from_millis(ms as u64);
    thread::sleep(time);
}
