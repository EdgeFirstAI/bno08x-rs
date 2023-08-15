use bno08x::interface::delay::{DelayMs, TimerMs};
use bno08x::wrapper::{
    BNO08x, SENSOR_REPORTID_ACCELEROMETER, SENSOR_REPORTID_GYROSCOPE,
    SENSOR_REPORTID_MAGNETIC_FIELD, SENSOR_REPORTID_ROTATION_VECTOR,
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
        BNO08x::new_bno08x_from_symbol("/dev/spidev1.0", "IMU_INT", "IMU_RST")?;

    let mut delay_source = TimerMs {};
    imu_driver.init(&mut delay_source).unwrap();

    let max_tries = 5;

    let reports = [
        (SENSOR_REPORTID_ROTATION_VECTOR, 100),
        (SENSOR_REPORTID_ACCELEROMETER, 300),
        (SENSOR_REPORTID_GYROSCOPE, 300),
        (SENSOR_REPORTID_MAGNETIC_FIELD, 300),
    ];
    for (r, t) in reports {
        let mut i = 0;
        while i < max_tries && !imu_driver.is_report_enabled(r) {
            imu_driver.enable_report(&mut delay_source, r, t).unwrap();
            i += 1;
        }

        if !imu_driver.is_report_enabled(r) {
            println!("Could not enable report {}", r);
            return Ok(());
        }
        delay_source.delay_ms(1000);
    }

    let loop_interval = 100;
    println!("loop_interval: {}", loop_interval);
    loop {
        let _msg_count = imu_driver.handle_messages(&mut delay_source, 10, 20);
        // if _msg_count > 0 {
        //     println!("> {}", _msg_count);
        // }
        delay_source.delay_ms(loop_interval);
        let [qi, qj, qk, qr] = imu_driver.rotation_quaternion().unwrap();
        println!(
            "Current rotation: {:?} time of last update: {}",
            quaternion_to_euler(qr, qi, qj, qk),
            imu_driver.report_update_time(SENSOR_REPORTID_ROTATION_VECTOR)
        );

        // let rot_acc: f32 = imu_driver.rotation_acc();
        // println!("Rotation Accuracy {}", rot_acc);

        // let [ax, ay, az] = imu_driver.accelerometer().unwrap();
        // println!("accelerometer (m/s^2): {} {} {}", ax, ay, az);

        // let [gx, gy, gz] = imu_driver.gyro().unwrap();
        // println!("gyroscope (rad/s): {} {} {}", gx, gy, gz);

        // let [mx, my, mz] = imu_driver.mag_field().unwrap();
        // println!("magnetometer (uTelsa): {} {} {}", mx, my, mz);
    }
    Ok(())
}
