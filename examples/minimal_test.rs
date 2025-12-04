use bno08x_rs::{BNO08x, SENSOR_REPORTID_ACCELEROMETER};
use std::{thread::sleep, time::Duration};

fn main() {
    env_logger::init();

    println!("Creating IMU...");
    let mut imu = BNO08x::new_spi_from_symbol("/dev/spidev1.0", "IMU_INT", "IMU_RST")
        .expect("Failed to create IMU driver");

    println!("Initializing IMU...");
    imu.init().expect("Failed to initialize IMU");

    println!("IMU initialized successfully!");
    sleep(Duration::from_millis(500));

    println!("Enabling accelerometer report...");
    match imu.enable_report(SENSOR_REPORTID_ACCELEROMETER, 100) {
        Ok(_) => println!("Accelerometer enabled"),
        Err(e) => println!("Failed to enable: {:?}", e),
    }

    sleep(Duration::from_millis(500));

    println!("Handling messages for 5 seconds...");
    for i in 0..50 {
        let count = imu.handle_one_message(200);
        println!("Iteration {}: handled {} messages", i, count);

        if count > 0 {
            match imu.accelerometer() {
                Ok(accel) => println!("  Accel: {:?}", accel),
                Err(e) => println!("  No accel data: {:?}", e),
            }
        }

        sleep(Duration::from_millis(100));
    }

    println!("Done!");
}
