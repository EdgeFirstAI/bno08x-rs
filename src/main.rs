use bno080::interface::{
    delay::{DelayMs, TimerMs},
};
use bno080::wrapper::BNO080;


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
        BNO080::new_bno080("/dev/spidev1.0", "/dev/gpiochip5", 2, 0)?;
    let mut delay_source = TimerMs {};
    imu_driver.init(&mut delay_source).unwrap();
    imu_driver.enable_rotation_vector(50).unwrap();

    let loop_interval = 50 as u8;
    println!("loop_interval: {}", loop_interval);

    loop {
        let _msg_count =
            imu_driver.handle_all_messages(&mut delay_source, 10u8);
        // if _msg_count > 0 {
        //     println!("> {}", _msg_count);
        // }
        delay_source.delay_ms(loop_interval);
        // println!("Current rotation: {:?}", imu_driver.rotation_quaternion());
        let [qr, qi, qj, qk] = imu_driver.rotation_quaternion().unwrap();
        println!(
            "Current rotation: {:?}",
            quaternion_to_euler(qr, qi, qj, qk)
        );
    }
}
