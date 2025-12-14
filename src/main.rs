//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use core::f32::consts::PI;
use cortex_m::prelude::_embedded_hal_adc_OneShot;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use embedded_hal::i2c::I2c; // Import the I2c trait for write_read
use embedded_hal::pwm::SetDutyCycle;
use libm::sinf;

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

mod current_sensor;

use current_sensor::CurrentSensor;

const TOP_VALUE: u16 = 4095;

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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

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

    //Setup ADC
    let adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);
    let adc_pin_0 = hal::adc::AdcPin::new(pins.gpio26).unwrap();
    let adc_pin_1 = hal::adc::AdcPin::new(pins.gpio27).unwrap();

    let mut current_sensor = CurrentSensor::new(adc, adc_pin_0, adc_pin_1);

    //Setup PWM

    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    let pwm_a = &mut pwm_slices.pwm1;
    pwm_a.set_ph_correct();
    pwm_a.set_top(TOP_VALUE);
    pwm_a.enable();
    let channel_a = &mut pwm_a.channel_a;
    channel_a.output_to(pins.gpio2);

    let pwm_b = &mut pwm_slices.pwm2;
    pwm_b.set_ph_correct();
    pwm_b.set_top(TOP_VALUE);
    pwm_b.enable();
    let channel_b = &mut pwm_b.channel_a;
    channel_b.output_to(pins.gpio4);

    let pwm_c = &mut pwm_slices.pwm3;
    pwm_c.set_ph_correct();
    pwm_c.set_top(TOP_VALUE);
    pwm_c.enable();
    let channel_c = &mut pwm_c.channel_a;
    channel_c.output_to(pins.gpio6);

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

    //set_angle(30.0, 90.0, channel_a, channel_b, channel_c);

    enable.set_high().unwrap();
    info!("Running");
    // Step the angle by 5 degrees up to and including 7*360 (2520)
    for d in (0..=(360 * 7)).step_by(5) {
        for j in (0..10) {
            current_sensor.read();
            current_sensor.display();
            delay.delay_us(200);
        }

        set_angle(80.0, d as f32, channel_a, channel_b, channel_c);

        for j in (0..10) {
            current_sensor.read();
            current_sensor.display();
            delay.delay_us(200);
        }
    }
    set_angle(90.0, 0 as f32, channel_a, channel_b, channel_c);
    delay.delay_ms(4);
    enable.set_low().unwrap();
    info!("stopped");
    // PWM configuration values

    let mut counter = 0u32;
    let mut start = timer.get_counter().ticks();
    let mut last_angle: f32 = 0.0;
    let mut current_angle: f32;

    let _angle = read_angle(&mut i2c).unwrap();
    info!("Sensor OK");
    loop {
        let current = timer.get_counter().ticks();

        if current.wrapping_sub(start) > 1_000_00 {
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

fn set_angle<A, B, C>(power: f32, angle: f32, ch_a: &mut A, ch_b: &mut B, ch_c: &mut C)
where
    A: SetDutyCycle,
    C: SetDutyCycle,
    B: SetDutyCycle,
{
    let angle_rad = angle.to_radians();

    let duty_scale = (TOP_VALUE as f32) / 2.0;

    let duty_a = (duty_scale + (power / 100.0) * sinf(angle_rad) * duty_scale) as u16;
    let duty_b =
        (duty_scale + (power / 100.0) * sinf(angle_rad - (2.0 * PI / 3.0)) * duty_scale) as u16;
    let duty_c =
        (duty_scale + (power / 100.0) * sinf(angle_rad + (2.0 * PI / 3.0)) * duty_scale) as u16;

    ch_a.set_duty_cycle(duty_a).unwrap();
    ch_b.set_duty_cycle(duty_b).unwrap();
    ch_c.set_duty_cycle(duty_c).unwrap();

    //info!("duty a {}, duty b {}, duty c {}", duty_a, duty_b, duty_c);
}
