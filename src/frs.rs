use bno08x::interface::delay::delay_ms;
use bno08x::wrapper::{BNO08x, SENSOR_REPORTID_ACCELEROMETER};
use std::io;

fn main() -> io::Result<()> {
    let mut imu_driver =
        BNO08x::new_bno08x_from_symbol("/dev/spidev1.0", "IMU_INT", "IMU_RST")?;

    imu_driver.init().unwrap();

    // Need to enable a report so that the IMU reports back to the program.
    let acc_enabled: bool = imu_driver
        .enable_report(SENSOR_REPORTID_ACCELEROMETER, 120)
        .unwrap();
    println!("Acceleration Enabled: {}", acc_enabled);

    let success = imu_driver.set_sensor_orientation(0.5, 0.5, 0.5, -0.5, 2000);
    Ok(())
}
