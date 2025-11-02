//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use embedded_hal::i2c::I2c; // Import the I2c trait for write_read
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal;
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{FunctionI2C, Pin},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use hal::Timer;

use hal::fugit::RateExtU32;

#[entry]
fn _start() -> ! {
    main()
}

fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let _delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    let mut enable = pins.gpio8.into_push_pull_output();
    enable.set_low().unwrap();

    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio20.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio21.reconfigure();

    let mut i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Test I2C initialization
    info!("I2C initialized successfully");

    // Initialize a counter for debugging
    let mut counter = 0u32;

    let mut start = timer.get_counter().ticks();

    let mut last_angle: f32 = 0.0;
    let mut current_angle: f32;

    let _angle = read_angle(&mut i2c).unwrap();
    info!("Sensor OK");

    loop {
        let current = timer.get_counter().ticks();

        if current.wrapping_sub(start) > 1_000_000 {
            start = current;
            info!("Current Time {}", current);
            led_pin.toggle().unwrap();
            info!("LED Toggle!");
            counter += 1;
            info!("Reached iteration {}", counter);
        }

        match read_angle(&mut i2c) {
            Ok(angle) => {
                current_angle = angle;
                if (last_angle - current_angle).abs() > 0.5 {
                    info!("Motor angle: {}", current_angle);
                    last_angle = current_angle;
                }
            }
            Err(_) => {
                info!("I2C read error occurred");
                // Continue with last known angle or handle error as needed
            }
        }
    }
}

fn read_angle<T: I2c>(i2c: &mut T) -> Result<f32, T::Error> {
    let mut buf = [0u8; 2];
    i2c.write_read(0x36u8, &[0x0E], &mut buf)?;
    let angle_u16 = ((buf[0] as u16) << 8) | (buf[1] as u16);
    Ok(angle_u16 as f32 / 4096.0 * 360.0)
}
// End of file
