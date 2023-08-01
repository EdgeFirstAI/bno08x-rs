use bno08x::interface::delay::{DelayMs, TimerMs};
use bno08x::wrapper::{
    BNO08x, SENSOR_REPORTID_ACCELEROMETER, SENSOR_REPORTID_GYROSCOPE,
    SENSOR_REPORTID_MAGNETIC_FIELD,
};

use std::{
    f32::consts::PI,
    io::{self},
};

const RAD_TO_DEG: f32 = 180f32 / PI;
fn quaternion_to_euler(qr: f32, qi: f32, qj: f32, qk: f32) -> [f32; 3] {
    let sqr = qr * qr;
    let sqi = qi * qi;
    let sqj = qj * qj;
    let sqk = qk * qk;

    let yaw =
        (2.0 * (qi * qj + qk * qr)).atan2(sqi - sqj - sqk + sqr) * RAD_TO_DEG;
    let pitch = (-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)).asin()
        * RAD_TO_DEG;
    let roll =
        (2.0 * (qj * qk + qi * qr)).atan2(-sqi - sqj + sqk + sqr) * RAD_TO_DEG;

    return [yaw, pitch, roll];
}

fn main() -> io::Result<()> {
    let mut imu_driver =
        BNO08x::new_bno08x("/dev/spidev1.0", "/dev/gpiochip5", 2, 0)?;
    let mut delay_source = TimerMs {};
    imu_driver.init(&mut delay_source).unwrap();
    imu_driver
        .enable_rotation_vector(&mut delay_source, 50)
        .unwrap();
    imu_driver
        .enable_report(&mut delay_source, SENSOR_REPORTID_ACCELEROMETER, 200)
        .unwrap();
    imu_driver
        .enable_report(&mut delay_source, SENSOR_REPORTID_GYROSCOPE, 200)
        .unwrap();
    imu_driver
        .enable_report(&mut delay_source, SENSOR_REPORTID_MAGNETIC_FIELD, 200)
        .unwrap();
    let loop_interval = 50;
    // println!("loop_interval: {}", loop_interval);
    loop {
        let _msg_count = imu_driver.handle_messages(&mut delay_source, 10, 10);
        // if _msg_count > 0 {
        //     println!("> {}", _msg_count);
        // }
        delay_source.delay_ms(loop_interval);
        let [qi, qj, qk, qr] = imu_driver.rotation_quaternion().unwrap();
        println!(
            "Current rotation: {:?}",
            quaternion_to_euler(qr, qi, qj, qk)
        );
        let [ax, ay, az] = imu_driver.accelerometer().unwrap();
        println!("accelerometer (m/s^2): {} {} {}", ax, ay, az);

        let [gx, gy, gz] = imu_driver.gyro().unwrap();
        println!("gyroscope (rad/s): {} {} {}", gx, gy, gz);

        let [mx, my, mz] = imu_driver.mag_field().unwrap();
        println!("magnetometer (uTelsa): {} {} {}", mx, my, mz);
    }
}
