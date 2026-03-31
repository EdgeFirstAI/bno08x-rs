// Copyright 2025 Au-Zone Technologies Inc.
// SPDX-License-Identifier: Apache-2.0

//! A simple watch channel for broadcasting value updates with timeout support.
//!
//! This provides a basic pub-sub mechanism where a sender can broadcast new
//! values and receivers can wait for changed values with an optional timeout.
//!
//! Built using only `std::sync::Mutex` and `std::sync::Condvar` - no external
//! dependencies.

use std::{
    sync::{Arc, Condvar, Mutex},
    time::Duration,
};

/// Error type for watch channel operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecvError {
    /// The receive operation timed out before a value change occurred
    Timeout,
    /// All senders have been dropped, channel is disconnected
    Disconnected,
}

impl std::fmt::Display for RecvError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RecvError::Timeout => write!(f, "recv_timeout: operation timed out"),
            RecvError::Disconnected => write!(f, "recv: channel disconnected, all senders dropped"),
        }
    }
}

impl std::error::Error for RecvError {}

/// Shared state for the watch channel
struct Shared<T> {
    value: Mutex<T>,
    condvar: Condvar,
    num_senders: Mutex<usize>,
}

/// Sends values to all active receivers
pub struct Sender<T> {
    shared: Arc<Shared<T>>,
}

impl<T> Clone for Sender<T> {
    fn clone(&self) -> Self {
        {
            let mut num = self.shared.num_senders.lock().expect("Mutex poisoned");
            *num += 1;
        }
        Sender {
            shared: Arc::clone(&self.shared),
        }
    }
}

impl<T> Drop for Sender<T> {
    fn drop(&mut self) {
        {
            let mut num = self.shared.num_senders.lock().expect("Mutex poisoned");
            *num -= 1;
        }
        // Wake up any waiting receivers so they can check if disconnected
        self.shared.condvar.notify_all();
    }
}

/// Receives values from a sender, with support for timeout-based waiting
#[derive(Clone)]
pub struct Receiver<T> {
    shared: Arc<Shared<T>>,
}

/// Creates a new watch channel with an initial value
pub fn channel<T: Clone>(initial: T) -> (Sender<T>, Receiver<T>) {
    let shared = Arc::new(Shared {
        value: Mutex::new(initial),
        condvar: Condvar::new(),
        num_senders: Mutex::new(1),
    });

    (
        Sender {
            shared: Arc::clone(&shared),
        },
        Receiver {
            shared: Arc::clone(&shared),
        },
    )
}

impl<T: Clone> Sender<T> {
    /// Sends a new value to all receivers and wakes them up
    pub fn send(&self, value: T) {
        {
            let mut state = self.shared.value.lock().expect("Mutex poisoned");
            *state = value;
        }
        // Notify all waiting receivers after releasing the lock
        self.shared.condvar.notify_all();
    }

    /// Returns a clone of the current value without waiting
    pub fn borrow(&self) -> T {
        self.shared.value.lock().expect("Mutex poisoned").clone()
    }
}

impl<T: Clone + PartialEq> Receiver<T> {
    /// Waits for the value to change (blocks indefinitely if value never
    /// changes)
    ///
    /// Returns `Ok(T)` if a new value is received, or
    /// `Err(RecvError::Disconnected)` if all senders are dropped.
    ///
    /// This will block until a new value is sent that differs from the current
    /// value.
    pub fn recv(&self) -> Result<T, RecvError> {
        loop {
            // Check if all senders are gone
            let num_senders = *self.shared.num_senders.lock().expect("Mutex poisoned");
            if num_senders == 0 {
                return Err(RecvError::Disconnected);
            }

            let old_value = self.shared.value.lock().expect("Mutex poisoned").clone();

            let guard = self.shared.value.lock().expect("Mutex poisoned");
            // Wait for notification while holding the lock
            let guard = self.shared.condvar.wait(guard).expect("Mutex poisoned");

            let new_value = (*guard).clone();
            drop(guard);

            // Only return if the value actually changed
            if new_value != old_value {
                return Ok(new_value);
            }
        }
    }

    /// Waits for the value to change with a timeout
    ///
    /// Returns `Ok(T)` if the value changed within the timeout duration,
    /// `Err(RecvError::Timeout)` if the timeout expired before a value
    /// change, or `Err(RecvError::Disconnected)` if all senders are dropped.
    pub fn recv_timeout(&self, duration: Duration) -> Result<T, RecvError> {
        let start = std::time::Instant::now();

        loop {
            // Check if all senders are gone
            let num_senders = *self.shared.num_senders.lock().expect("Mutex poisoned");
            if num_senders == 0 {
                return Err(RecvError::Disconnected);
            }

            let old_value = self.shared.value.lock().expect("Mutex poisoned").clone();
            let elapsed = start.elapsed();

            if elapsed >= duration {
                return Err(RecvError::Timeout);
            }

            let remaining = duration - elapsed;

            let guard = self.shared.value.lock().expect("Mutex poisoned");
            let (guard, wait_result) = self
                .shared
                .condvar
                .wait_timeout(guard, remaining)
                .expect("Mutex poisoned");

            let new_value = (*guard).clone();
            drop(guard);

            // Return if value changed
            if new_value != old_value {
                return Ok(new_value);
            }

            // If timeout occurred, return error
            if wait_result.timed_out() {
                return Err(RecvError::Timeout);
            }

            // Otherwise loop to wait again (spurious wakeup or multiple
            // senders)
        }
    }

    /// Returns a clone of the current value without waiting
    pub fn borrow(&self) -> T {
        self.shared.value.lock().expect("Mutex poisoned").clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{thread, time::Duration};

    #[test]
    fn test_send_recv() {
        let (tx, rx) = channel(0);

        let handle = thread::spawn({
            let tx = tx.clone();
            move || {
                thread::sleep(Duration::from_millis(50));
                tx.send(42);
            }
        });

        let value = rx.recv();
        assert_eq!(value, Ok(42));
        handle.join().unwrap();
    }

    #[test]
    fn test_recv_timeout_success() {
        let (tx, rx) = channel(0);

        let handle = thread::spawn({
            let tx = tx.clone();
            move || {
                thread::sleep(Duration::from_millis(50));
                tx.send(100);
            }
        });

        let result = rx.recv_timeout(Duration::from_millis(500));
        assert_eq!(result, Ok(100));
        handle.join().unwrap();
    }

    #[test]
    fn test_recv_timeout_expired() {
        let (_tx, rx) = channel::<i32>(0);

        let result = rx.recv_timeout(Duration::from_millis(50));
        assert_eq!(result, Err(RecvError::Timeout));
    }

    #[test]
    fn test_multiple_receivers() {
        let (tx, rx1) = channel(0);
        let rx2 = rx1.clone();
        let rx3 = rx1.clone();

        let handle1 = thread::spawn(move || rx1.recv());
        let handle2 = thread::spawn(move || rx2.recv());
        let handle3 = thread::spawn(move || rx3.recv());

        thread::sleep(Duration::from_millis(50));
        tx.send(10);

        let v1 = handle1.join().unwrap();
        let v2 = handle2.join().unwrap();
        let v3 = handle3.join().unwrap();

        assert_eq!(v1, Ok(10));
        assert_eq!(v2, Ok(10));
        assert_eq!(v3, Ok(10));
    }

    #[test]
    fn test_borrow() {
        let (tx, rx) = channel(5);

        assert_eq!(rx.borrow(), 5);
        assert_eq!(tx.borrow(), 5);

        tx.send(15);

        assert_eq!(rx.borrow(), 15);
        assert_eq!(tx.borrow(), 15);
    }

    #[test]
    fn test_value_must_change() {
        let (tx, rx) = channel(42);

        let handle = thread::spawn(move || {
            // Send same value - recv should not return
            tx.send(42);
            thread::sleep(Duration::from_millis(100));
            // Send different value - now recv should return
            tx.send(99);
        });

        let value = rx.recv_timeout(Duration::from_millis(500));
        assert!(
            value.is_ok() && value.unwrap() == 99,
            "Should get 99, not 42"
        );
        handle.join().unwrap();
    }

    #[test]
    fn test_multiple_updates() {
        let (tx, rx) = channel(0);

        let handle = thread::spawn(move || {
            tx.send(1);
            thread::sleep(Duration::from_millis(10));
            tx.send(2);
            thread::sleep(Duration::from_millis(10));
            tx.send(3);
        });

        assert_eq!(rx.recv(), Ok(1));
        assert_eq!(rx.recv(), Ok(2));
        assert_eq!(rx.recv(), Ok(3));
        handle.join().unwrap();
    }

    #[test]
    fn test_disconnected_on_recv() {
        let (tx, rx) = channel(0);

        let handle = thread::spawn({
            let tx = tx.clone();
            move || {
                thread::sleep(Duration::from_millis(50));
                tx.send(99); // Send a value to wake up waiters
                drop(tx); // Then drop the sender
            }
        });

        // Recv should get the sent value
        assert_eq!(rx.recv(), Ok(99));
        handle.join().unwrap();

        // Drop the original sender so num_senders reaches 0
        drop(tx);

        // Now all senders are gone, next recv should see disconnected
        assert_eq!(rx.recv(), Err(RecvError::Disconnected));
    }

    #[test]
    fn test_disconnected_timeout() {
        let (tx, rx) = channel::<i32>(0);
        drop(tx); // Drop the only sender

        // Should immediately get Disconnected, not timeout
        let result = rx.recv_timeout(Duration::from_millis(500));
        assert_eq!(result, Err(RecvError::Disconnected));
    }
}
