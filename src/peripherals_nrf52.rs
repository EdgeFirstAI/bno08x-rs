// use nrf52832_hal as p_hal;
// use p_hal::nrf52832_pac as pac;
// use p_hal::{
//     clocks::ClocksExt,
//     gpio::{GpioExt, Level},
// };

// use p_hal::{delay::Delay, spim, twim};

// use bno080::interface::spi::SpiControlLines;
// use embedded_hal::blocking::delay::DelayMs;
// use embedded_hal::digital::v2::{toggleable, OutputPin, StatefulOutputPin};

// pub type Spi1PortType = p_hal::spim::Spim<pac::SPIM0>;

// type ChipSelectPinType =
//     p_hal::gpio::Pin<p_hal::gpio::Output<p_hal::gpio::PushPull>>; //CSN
// type HIntPinType = p_hal::gpio::Pin<p_hal::gpio::Input<p_hal::gpio::Floating>>; //HINTN
// type ResetPinType =
//     p_hal::gpio::Pin<p_hal::gpio::Output<p_hal::gpio::PushPull>>; // RESET

// pub type BnoSpi1Lines =
//     SpiControlLines<Spi1PortType, ChipSelectPinType, HIntPinType, ResetPinType>;

// type InnerUserLed1Type =
//     p_hal::gpio::p0::P0_17<p_hal::gpio::Output<p_hal::gpio::PushPull>>;
// type UserLed1Type = ToggleThing<InnerUserLed1Type>;

// pub struct ToggleThing<T> {
//     inner: T,
// }

// impl<T> StatefulOutputPin for ToggleThing<T>
// where
//     T: StatefulOutputPin + OutputPin,
// {
//     fn is_set_high(&self) -> Result<bool, Self::Error> {
//         self.inner.is_set_high()
//     }

//     fn is_set_low(&self) -> Result<bool, Self::Error> {
//         self.inner.is_set_low()
//     }
// }

// impl<T> OutputPin for ToggleThing<T>
// where
//     T: StatefulOutputPin + OutputPin,
// {
//     type Error = T::Error;

//     fn set_low(&mut self) -> Result<(), Self::Error> {
//         self.inner.set_low()
//     }

//     fn set_high(&mut self) -> Result<(), Self::Error> {
//         self.inner.set_high()
//     }
// }

// impl<T> toggleable::Default for ToggleThing<T> where
//     T: StatefulOutputPin + OutputPin
// {
// }

// pub fn setup_peripherals(
// ) -> (UserLed1Type, impl DelayMs, ImuI2cPortType, BnoSpi1Lines) {
//     let cp = pac::CorePeripherals::take().unwrap();
//     let delay_source = Delay::new(cp.SYST);

//     // PineTime has a 32 MHz HSE (HFXO) and a 32.768 kHz LSE (LFXO)
//     let dp = pac::Peripherals::take().unwrap();
//     let _clockit = dp.CLOCK.constrain().start_lfclk().enable_ext_hfosc();

//     let port0 = dp.P0.split();

//     // random number generator peripheral
//     //let mut rng = dp.RNG.constrain();
//     let user_led1: UserLed1Type = ToggleThing {
//         inner: port0.p0_17.into_push_pull_output(Level::High),
//     };

//     let i2c0_pins = twim::Pins {
//         scl: port0.p0_07.into_floating_input().degrade(),
//         sda: port0.p0_06.into_floating_input().degrade(),
//     };
//     let i2c_port = twim::Twim::new(dp.TWIM1, i2c0_pins, twim::Frequency::K400);

//     let spi_ctrl_lines = {
//         let spim0_pins = spim::Pins {
//             sck: port0.p0_02.into_push_pull_output(Level::Low).degrade(),
//             miso: Some(port0.p0_27.into_floating_input().degrade()),
//             mosi: Some(port0.p0_26.into_push_pull_output(Level::Low).degrade()),
//         };

//         // create SPIM0 interface, 8 Mbps, use 122 as "over read character"
//         let spim0 = spim::Spim::new(
//             dp.SPIM0,
//             spim0_pins,
//             spim::Frequency::M4,
//             spim::MODE_3,
//             122,
//         );

//         // SPI chip select CS
//         let csn = port0.p0_25.into_push_pull_output(Level::High).degrade();

//         // HINTN interrupt pin
//         let hintn = port0.p0_24.into_floating_input().degrade();

//         // NRSTN pin
//         let reset_pin =
//             port0.p0_23.into_push_pull_output(Level::High).degrade();

//         SpiControlLines {
//             spi: spim0,
//             csn,
//             hintn,
//             reset: reset_pin,
//         }
//     };

//     (user_led1, delay_source, i2c_port, spi_ctrl_lines)
// }

// type ImuI2cPortType = p_hal::twim::Twim<pac::TWIM1>;
