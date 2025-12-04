// Copyright 2025 Au-Zone Technologies Inc.
// SPDX-License-Identifier: Apache-2.0

use bno08x_rs::{
    interface::{
        delay::delay_ms,
        gpio::{GpiodIn, GpiodOut},
        spidev::SpiDevice,
        SpiInterface,
    },
    BNO08x, SENSOR_REPORTID_ACCELEROMETER, SENSOR_REPORTID_GYROSCOPE,
    SENSOR_REPORTID_MAGNETIC_FIELD, SENSOR_REPORTID_ROTATION_VECTOR,
};
use std::{f32::consts::PI, io};

const RAD_TO_DEG: f32 = 180f32 / PI;

// https://stackoverflow.com/a/37560411
fn quaternion_to_euler(qr: f32, qi: f32, qj: f32, qk: f32) -> [f32; 3] {
    let yaw = (2.0 * (qk * qr + qi * qj)).atan2(-1.0 + 2.0 * (qr * qr + qi * qi)) * RAD_TO_DEG;
    let pitch = (2.0 * (qj * qr - qk * qi)).asin() * RAD_TO_DEG;
    let roll = (2.0 * (qk * qj + qr * qi)).atan2(1.0 - 2.0 * (qi * qi + qj * qj)) * RAD_TO_DEG;

    [yaw, pitch, roll]
}

fn print_info(imu_driver: &BNO08x<SpiInterface<SpiDevice, GpiodIn, GpiodOut>>) {
    let [qi, qj, qk, qr] = imu_driver.rotation_quaternion().unwrap();
    let [yaw, pitch, roll] = quaternion_to_euler(qr, qi, qj, qk);
    let [ax, ay, az] = imu_driver.accelerometer().unwrap();
    let [gx, gy, gz] = imu_driver.gyro().unwrap();
    let [mx, my, mz] = imu_driver.mag_field().unwrap();

    let attitude_message = format!(
        "Attitude [degrees]: yaw={:.3}, pitch={:.3}, roll={:.3}, accuracy={:.3}",
        yaw,
        pitch,
        roll,
        imu_driver.rotation_acc()
    );

    let accelerometer_message = format!(
        "Accelerometer [m/s^2]: ax={:.3}, ay={:.3}, az={:.3}",
        ax, ay, az
    );

    let gyroscope_message = format!(
        "Gyroscope [rad/s]: gx={:.3}, gy={:.3}, gz={:.3}",
        gx, gy, gz
    );

    let magnetometer_message = format!(
        "Magnetometer [uTesla]: mx={:.3}, my={:.3}, mz={:.3}",
        mx, my, mz
    );

    let timestamp_message = format!(
        "timestamp [ns]: {}",
        imu_driver.report_update_time(SENSOR_REPORTID_ROTATION_VECTOR)
    );
    let update = format!(
        "{}\n{}\n{}\n{}\n{}\n",
        attitude_message,
        accelerometer_message,
        gyroscope_message,
        magnetometer_message,
        timestamp_message
    );

    println!("{}", update);
}

fn main() -> io::Result<()> {
    let mut imu_driver = BNO08x::new_spi_from_symbol("/dev/spidev1.0", "IMU_INT", "IMU_RST")?;

    imu_driver.init().unwrap();

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
            imu_driver.enable_report(r, t).unwrap();
            i += 1;
        }

        if !imu_driver.is_report_enabled(r) {
            println!("Could not enable report {}", r);
            return Ok(());
        }
        println!("Report {} is enabled", r);
        delay_ms(1000);
    }
    imu_driver.add_sensor_report_callback(
        SENSOR_REPORTID_ROTATION_VECTOR,
        String::from("print_info"),
        print_info,
    );
    let loop_interval = 50;
    println!("loop_interval: {}", loop_interval);
    loop {
        let _msg_count = imu_driver.handle_messages(10, 20);
        delay_ms(loop_interval);
    }
}
