extern crate gpiod;
extern crate spidev;
use gpiod::{AsValuesMut, Chip, EdgeDetect, Masked, Options};
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::io::prelude::*;

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

fn main() {}
