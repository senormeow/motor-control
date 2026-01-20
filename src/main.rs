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
use rp_pico::pac::pwm::ch;

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

//const TOP_VALUE: u16 = 4095;
const TOP_VALUE: u16 = 4094 - 1;
// Deadtime in PWM ticks (e.g., 100 ticks ~ 0.8us at 125MHz system clock with divider 1)
const DEADTIME_TICKS: u16 = 100;

/// Motor controller struct that encapsulates all 6 PWM channels for 3-phase motor control
pub struct Motor<A, B, C, An, Bn, Cn>
where
    A: SetDutyCycle,
    B: SetDutyCycle,
    C: SetDutyCycle,
    An: SetDutyCycle,
    Bn: SetDutyCycle,
    Cn: SetDutyCycle,
{
    ch_a: A,
    ch_b: B,
    ch_c: C,
    ch_an: An,
    ch_bn: Bn,
    ch_cn: Cn,
    // Open-loop control state
    shaft_angle: f32,
    open_loop_timestamp: u64,
    // Motor parameters
    pole_pairs: u8,
    voltage_limit: f32,
}

impl<A, B, C, An, Bn, Cn> Motor<A, B, C, An, Bn, Cn>
where
    A: SetDutyCycle,
    B: SetDutyCycle,
    C: SetDutyCycle,
    An: SetDutyCycle,
    Bn: SetDutyCycle,
    Cn: SetDutyCycle,
{
    /// Create a new Motor instance with all 6 PWM channels
    /// pole_pairs: number of motor pole pairs
    /// voltage_limit: maximum voltage (as duty cycle 0.0-1.0)
    pub fn new(
        ch_a: A,
        ch_b: B,
        ch_c: C,
        ch_an: An,
        ch_bn: Bn,
        ch_cn: Cn,
        pole_pairs: u8,
        voltage_limit: f32,
    ) -> Self {
        Self {
            ch_a,
            ch_b,
            ch_c,
            ch_an,
            ch_bn,
            ch_cn,
            shaft_angle: 0.0,
            open_loop_timestamp: 0,
            pole_pairs,
            voltage_limit,
        }
    }

    /// Set the PWM duty cycles for all three phases with deadtime compensation
    /// duty_a, duty_b, duty_c should be floats between 0.0 and 1.0
    pub fn set_pwm(&mut self, duty_a: f32, duty_b: f32, duty_c: f32) {
        // Convert float (0.0-1.0) to u16 (0-TOP_VALUE)
        let duty_a_raw = (duty_a.clamp(0.0, 1.0) * TOP_VALUE as f32) as u16;
        let duty_b_raw = (duty_b.clamp(0.0, 1.0) * TOP_VALUE as f32) as u16;
        let duty_c_raw = (duty_c.clamp(0.0, 1.0) * TOP_VALUE as f32) as u16;

        // Apply deadtime: reduce high-side duty, increase low-side (inverted) duty
        // This creates a gap where both sides are off
        let duty_a_hs = duty_a_raw.saturating_sub(DEADTIME_TICKS / 2);
        let duty_a_ls = duty_a_raw.saturating_add(DEADTIME_TICKS / 2).min(TOP_VALUE);
        let duty_b_hs = duty_b_raw.saturating_sub(DEADTIME_TICKS / 2);
        let duty_b_ls = duty_b_raw.saturating_add(DEADTIME_TICKS / 2).min(TOP_VALUE);
        let duty_c_hs = duty_c_raw.saturating_sub(DEADTIME_TICKS / 2);
        let duty_c_ls = duty_c_raw.saturating_add(DEADTIME_TICKS / 2).min(TOP_VALUE);

        self.ch_a.set_duty_cycle(duty_a_hs).unwrap();
        self.ch_an.set_duty_cycle(duty_a_ls).unwrap();
        self.ch_b.set_duty_cycle(duty_b_hs).unwrap();
        self.ch_bn.set_duty_cycle(duty_b_ls).unwrap();
        self.ch_c.set_duty_cycle(duty_c_hs).unwrap();
        self.ch_cn.set_duty_cycle(duty_c_ls).unwrap();
    }

    /// Set motor angle using SVPWM (Space Vector PWM)
    /// power: motor power as float between 0.0 and 1.0
    /// angle: electrical angle in degrees
    pub fn set_angle(&mut self, power: f32, angle: f32) {
        let angle_rad = angle.to_radians();

        // Calculate three-phase sine values
        let sin_a = sinf(angle_rad);
        let sin_b = sinf(angle_rad - (2.0 * PI / 3.0));
        let sin_c = sinf(angle_rad + (2.0 * PI / 3.0));

        // Apply space vector modulation offset to increase voltage utilization
        // This adds the average of min and max to center the waveform
        let max_val = sin_a.max(sin_b).max(sin_c);
        let min_val = sin_a.min(sin_b).min(sin_c);
        let offset = -(max_val + min_val) / 2.0;

        // Calculate duty cycle as float (0.0-1.0) with SVPWM
        let duty_a = 0.5 + power * (sin_a + offset) * 0.5;
        let duty_b = 0.5 + power * (sin_b + offset) * 0.5;
        let duty_c = 0.5 + power * (sin_c + offset) * 0.5;

        self.set_pwm(duty_a, duty_b, duty_c);
    }

    /// Open-loop velocity control
    /// target_velocity: target velocity in rad/s
    /// now_us: current timestamp in microseconds
    /// Returns the applied voltage (duty cycle)
    pub fn velocity_openloop(&mut self, target_velocity: f32, now_us: u64) -> f32 {
        // Calculate the sample time from last call
        let dt_us = now_us.wrapping_sub(self.open_loop_timestamp);
        let mut ts = dt_us as f32 * 1e-6;

        // Quick fix for strange cases (overflow + timestamp not defined)
        if ts <= 0.0 || ts > 0.5 {
            ts = 1e-3;
        }

        // Calculate the necessary angle to achieve target velocity
        self.shaft_angle = normalize_angle(self.shaft_angle + target_velocity * ts);

        // Calculate electrical angle from mechanical angle
        let electrical_angle = self.shaft_angle * self.pole_pairs as f32;

        // Set phase voltage with the necessary electrical angle
        self.set_angle(self.voltage_limit, electrical_angle.to_degrees());

        // Save timestamp for next call
        self.open_loop_timestamp = now_us;

        self.voltage_limit
    }
}

/// Normalize angle to [0, 2*PI) range
fn normalize_angle(angle: f32) -> f32 {
    let mut a = angle % (2.0 * PI);
    if a < 0.0 {
        a += 2.0 * PI;
    }
    a
}

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
    pwm_a.channel_b.set_inverted();
    pwm_a.channel_a.output_to(pins.gpio2);
    pwm_a.channel_b.output_to(pins.gpio3);

    let pwm_b = &mut pwm_slices.pwm2;
    pwm_b.set_ph_correct();
    pwm_b.set_top(TOP_VALUE);
    pwm_b.enable();
    pwm_b.channel_b.set_inverted();
    pwm_b.channel_a.output_to(pins.gpio4);
    pwm_b.channel_b.output_to(pins.gpio5);

    let pwm_c = &mut pwm_slices.pwm3;
    pwm_c.set_ph_correct();
    pwm_c.set_top(TOP_VALUE);
    pwm_c.enable();
    pwm_c.channel_b.set_inverted();
    pwm_c.channel_a.output_to(pins.gpio6);
    pwm_c.channel_b.output_to(pins.gpio7);

    let mut motor = Motor::new(
        &mut pwm_a.channel_a,
        &mut pwm_b.channel_a,
        &mut pwm_c.channel_a,
        &mut pwm_a.channel_b,
        &mut pwm_b.channel_b,
        &mut pwm_c.channel_b,
        7,   // pole_pairs (adjust for your motor)
        0.8, // voltage_limit (0.0-1.0)
    );

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
    let target_velocity: f32 = 12.0; // rad/s (adjust as needed)

    //let mut start = timer.get_counter().ticks();
    let mut last_angle: f32 = 0.0;
    let mut current_angle: f32;
    //let mut counter = 0u32;
    loop {
        let current = timer.get_counter().ticks();
        motor.velocity_openloop(target_velocity, current);
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

    // //let _angle = read_angle(&mut i2c).unwrap();
    // info!("Sensor OK");
    // loop {
    //     let current = timer.get_counter().ticks();

    //     if current.wrapping_sub(start) > 1_000_00 {
    //         start = current;
    //         info!("Current Time {}", current);
    //         led_pin.toggle().unwrap();
    //         info!("LED Toggle!");
    //         counter += 1;
    //         info!("Reached iteration {}", counter);
    //     }

    //     // match read_angle(&mut i2c) {
    //     //     Ok(angle) => {
    //     //         current_angle = angle;
    //     //         if (last_angle - current_angle).abs() > 0.5 {
    //     //             info!("Motor angle: {}", current_angle);
    //     //             last_angle = current_angle;
    //     //         }
    //     //     }
    //     //     Err(_) => {
    //     //         info!("I2C read error occurred");
    //     //         // Continue with last known angle or handle error as needed
    //     //     }
    //     // }
    // }
}

fn read_angle<T: I2c>(i2c: &mut T) -> Result<f32, T::Error> {
    let mut buf = [0u8; 2];
    i2c.write_read(0x36u8, &[0x0E], &mut buf)?;
    let angle_u16 = ((buf[0] as u16) << 8) | (buf[1] as u16);
    Ok(angle_u16 as f32 / 4096.0 * 360.0)
}
