//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
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

use hal::fugit::RateExtU32;

#[entry]
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

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

    loop {
        info!("Loop iteration: {}", counter);

        info!("on!");
        led_pin.set_high().unwrap();
        delay.delay_ms(500);

        info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(500);

        counter += 1;

        let angle = read_angle(&mut i2c);
        info!("Motor angle: {:?} degrees", angle);

        // Add a breakpoint condition every 10 iterations
        if counter % 10 == 0 {
            info!("Reached iteration {}", counter);
        }
    }
}

fn read_angle<T: I2c>(i2c: &mut T) -> f32 {
    let mut buf = [0u8; 2];
    i2c.write_read(0x36u8, &[0x0E], &mut buf).unwrap();
    let angle_u16 = ((buf[0] as u16) << 8) | (buf[1] as u16);
    angle_u16 as f32 / 4096.0 * 360.0
}
// End of file
