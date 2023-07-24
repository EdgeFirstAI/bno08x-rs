// Export the macro
#[macro_export]
// Disable warnings
#[allow(unused_macros)]
// The debug version
#[cfg(debug_assertions)]
macro_rules! log {
    ($( $args:expr ),*) => { println!( $( $args ),* ); }
}

// Non-debug version

#[cfg(not(debug_assertions))]
macro_rules! log {
    ($( $args:expr ),*) => {
        ()
    };
}
