use bno080::interface::{
    delay::{DelayMs, TimerMs},
    gpio::{GpiodIn, GpiodOut},
    spi::SpiControlLines,
    spidev::{SpiDevice, Transfer, Write},
    SpiInterface,
};
use bno080::wrapper::BNO080;
use gpiod::{AsValuesMut, Chip, EdgeDetect, Masked, Options};
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::{
    f32::consts::PI,
    io::{self, prelude::*},
};
// use bno080::wrapper::BNO080;
// // use bno080::interface::{I2cInterface, SpiInterface};
// use embedded_hal::blocking::delay::DelayMs;
// use embedded_hal::digital::v2::OutputPin;
// use embedded_hal::digital::v2::ToggleableOutputPin;

// const IMU_REPORTING_RATE_HZ: u16 = 200;
// const IMU_REPORTING_INTERVAL_MS: u16 = (1000 / IMU_REPORTING_RATE_HZ);

// // type ImuDriverType = bno080::wrapper::BNO080<I2cInterface<ImuI2cPortType>>;
// type ImuDriverType = bno080::wrapper::BNO080<
//     SpiInterface<
//         SpiPortType,
//         ChipSelectPinType,
//         HIntPinType,
//         WakePinType,
//         ResetPinType,
//     >,
// >;

// fn main() -> ! {
//     let (mut user_led1, mut delay_source, _i2c_port, mut _spi_control_lines) =
//         peripherals::setup_peripherals();

//     // SPI interface
//     let iface = bno080::interface::SpiInterface::new(_spi_control_lines);

//     // I2C interface
//     // let iface = bno080::interface::I2cInterface::default(_i2c_port);

//     let mut imu_driver = BNO080::new_with_interface(iface);
//     imu_driver.init(&mut delay_source).unwrap();

//     //cortex_m::asm::bkpt();
//     imu_driver
//         .enable_rotation_vector(IMU_REPORTING_INTERVAL_MS)
//         .unwrap();

//     let loop_interval = IMU_REPORTING_INTERVAL_MS as u8;
//     println!("loop_interval: {}", loop_interval);

//     let _ = user_led1.set_low();

//     loop {
//         let _msg_count = imu_driver.handle_all_messages(&mut delay_source, 1u8);
//         if _msg_count > 0 {
//             println!("> {}", _msg_count);
//         }

//         let _ = user_led1.toggle();
//         delay_source.delay_ms(loop_interval);
//     }
// }
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
    let mut chip = Chip::new("/dev/gpiochip5")?; // open chip
    let mut spi = SpiDevice::new("/dev/spidev1.0")?;
    let ctrl_lines: SpiControlLines<SpiDevice, GpiodIn, GpiodOut> =
        SpiControlLines::<SpiDevice, GpiodIn, GpiodOut> {
            spi: spi,
            // csn: (),
            hintn: GpiodIn::new(&chip, 2)?,
            reset: GpiodOut::new(&chip, 0)?,
        };

    let spi_int = SpiInterface::new(ctrl_lines);

    let mut delay_source = TimerMs {};

    let mut imu_driver = BNO080::new_with_interface(spi_int);
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
