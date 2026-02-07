//! BLDC Motor Controller for RP Pico
//!
//! Implements closed-loop velocity control using sinusoidal PWM commutation
//! with AS5600 magnetic encoder feedback.
#![no_std]
#![no_main]

use bsp::entry;
use core::f32::consts::PI;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;
use embedded_hal::pwm::SetDutyCycle;
use libm::sinf;

use panic_probe as _;

// Bring the embedded-hal 0.2 OneShot trait into scope (rp2040-hal expects this).
use cortex_m::prelude::_embedded_hal_adc_OneShot;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal;
use hal::{
    clocks::init_clocks_and_plls,
    gpio::{FunctionI2C, Pin},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use hal::fugit::RateExtU32;

//const TOP_VALUE: u16 = 4095;
const TOP_VALUE: u16 = 8191; // 13-bit resolution for finer control (adjust as needed)
                             // Deadtime in PWM ticks (e.g., 100 ticks ~ 0.8us at 125MHz system clock with divider 1)
const DEADTIME_TICKS: u16 = 200;

// V/f curve constants for Nissan EM57 at 30V bus
// Derived from empirical data: Vlim ≈ VF_SLOPE * velocity + VF_OFFSET
// Fits measured points from 8.9–44.5 rad/s within ~1%
const VF_SLOPE: f32 = 0.0201; // duty-cycle per rad/s
const VF_OFFSET: f32 = 0.016; // minimum duty-cycle at zero speed

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
    // Closed-loop control state
    prev_shaft_angle: f32,
    shaft_velocity: f32,
    velocity_integral: f32,
    last_timestamp: u64,
    fast_timestamp: u64,
    last_voltage: f32,
    // Motor parameters
    pole_pairs: u8,
    voltage_limit: f32,
    // PI controller gains
    velocity_p: f32,
    velocity_i: f32,
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
    /// velocity_p: proportional gain for velocity PI controller
    /// velocity_i: integral gain for velocity PI controller
    pub fn new(
        ch_a: A,
        ch_b: B,
        ch_c: C,
        ch_an: An,
        ch_bn: Bn,
        ch_cn: Cn,
        pole_pairs: u8,
        voltage_limit: f32,
        velocity_p: f32,
        velocity_i: f32,
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
            prev_shaft_angle: 0.0,
            shaft_velocity: 0.0,
            velocity_integral: 0.0,
            last_timestamp: 0,
            fast_timestamp: 0,
            last_voltage: 0.0,
            pole_pairs,
            voltage_limit,
            velocity_p,
            velocity_i,
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

        // Calculate three-phase sine values (a-c-b sequence)
        let sin_a = sinf(angle_rad);
        let sin_b = sinf(angle_rad - (2.0 * PI / 3.0));
        let sin_c = sinf(angle_rad + (2.0 * PI / 3.0));

        // Apply space vector modulation offset to increase voltage utilization
        // This adds the average of min and max to center the waveform (midpoint clamp)
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

    /// Closed-loop velocity control with PI controller
    /// target_velocity: target velocity in rad/s
    /// shaft_angle_deg: current shaft angle from encoder in degrees
    /// now_us: current timestamp in microseconds
    /// Returns the applied voltage (duty cycle)
    pub fn velocity_closedloop(
        &mut self,
        target_velocity: f32,
        shaft_angle_deg: f32,
        now_us: u64,
    ) -> f32 {
        // Convert shaft angle to radians
        let shaft_angle_rad = shaft_angle_deg.to_radians();

        // Calculate time delta
        let dt_us = now_us.wrapping_sub(self.last_timestamp);
        let mut dt = dt_us as f32 * 1e-6;

        // Quick fix for strange cases (overflow + timestamp not defined)
        if dt <= 0.0 || dt > 0.5 {
            dt = 1e-3;
        }

        // Calculate angle difference (handle wraparound)
        let mut angle_diff = shaft_angle_rad - self.prev_shaft_angle;
        if angle_diff > PI {
            angle_diff -= 2.0 * PI;
        } else if angle_diff < -PI {
            angle_diff += 2.0 * PI;
        }

        // Calculate actual velocity (lighter filtering for faster response)
        let measured_velocity = angle_diff / dt;
        self.shaft_velocity = 0.8 * self.shaft_velocity + 0.2 * measured_velocity;

        // Velocity error
        let velocity_error = target_velocity - self.shaft_velocity;

        // PI controller
        self.velocity_integral += velocity_error * dt;
        // Anti-windup: clamp integral
        self.velocity_integral = self.velocity_integral.clamp(
            -1.0 / self.velocity_i.max(0.001),
            1.0 / self.velocity_i.max(0.001),
        );

        let voltage = self.velocity_p * velocity_error + self.velocity_i * self.velocity_integral;
        let voltage_clamped = voltage.clamp(-self.voltage_limit, self.voltage_limit);

        // No delay compensation at low speeds - only enable above threshold
        let predicted_angle = if self.shaft_velocity.abs() > 25.0 {
            let loop_delay_compensation = 0.0007; // ~700μs compensation
            shaft_angle_rad + self.shaft_velocity * loop_delay_compensation
        } else {
            shaft_angle_rad
        };

        // Calculate electrical angle from predicted mechanical angle
        // Phase lead direction based on voltage sign (torque direction we want to apply)
        // Negative sign because motor winding direction is reversed
        let direction = if voltage_clamped >= 0.0 { -1.0 } else { 1.0 };
        let electrical_angle = predicted_angle * self.pole_pairs as f32 + direction * PI / 2.0;

        // Set phase voltage using absolute value
        self.set_angle(voltage_clamped.abs(), electrical_angle.to_degrees());

        // Save state for next call
        self.prev_shaft_angle = shaft_angle_rad;
        self.last_timestamp = now_us;
        self.fast_timestamp = now_us;
        self.last_voltage = voltage_clamped;

        voltage_clamped
    }

    /// Fast angle update using predicted angle (call between I2C reads)
    /// Uses last measured velocity to extrapolate angle
    pub fn update_angle_fast(&mut self, now_us: u64) {
        // Calculate time since last fast update
        let dt_us = now_us.wrapping_sub(self.fast_timestamp);
        let dt = dt_us as f32 * 1e-6;

        if dt <= 0.0 || dt > 0.01 {
            self.fast_timestamp = now_us;
            return; // Skip if time is invalid or too long
        }

        // Predict current angle using velocity (cumulative from prev_shaft_angle)
        let time_since_measurement = (now_us.wrapping_sub(self.last_timestamp)) as f32 * 1e-6;
        let predicted_angle = self.prev_shaft_angle + self.shaft_velocity * time_since_measurement;

        // Calculate electrical angle
        let direction = if self.last_voltage >= 0.0 { -1.0 } else { 1.0 };
        let electrical_angle = predicted_angle * self.pole_pairs as f32 + direction * PI / 2.0;

        // Update PWM with same voltage but predicted angle
        self.set_angle(self.last_voltage.abs(), electrical_angle.to_degrees());

        self.fast_timestamp = now_us;
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

/// Compute voltage limit from V/f curve for a given velocity
/// Returns duty-cycle (0.0–1.0) based on the empirical Nissan EM57 V/f relationship
fn velocity_to_voltage(velocity: f32) -> f32 {
    (VF_SLOPE * velocity.abs() + VF_OFFSET).clamp(0.0, 1.0)
}

#[entry]
fn _start() -> ! {
    main()
}

fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
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

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let _led_pin = pins.led.into_push_pull_output();

    let mut enable = pins.gpio8.into_push_pull_output();
    enable.set_low().unwrap();

    //Setup ADC
    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_pin_0 = hal::adc::AdcPin::new(pins.gpio26).unwrap();
    let mut adc_pin_1 = hal::adc::AdcPin::new(pins.gpio27).unwrap();

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
        4,    // pole_pairs (adjust for your motor)
        0.5,  // voltage_limit (0.0-1.0) - max power
        0.08, // velocity_p (proportional gain)
        0.02, // velocity_i (integral gain)
    );

    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio20.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio21.reconfigure();

    let mut i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        1000.kHz(), // Fast mode plus (1MHz) for faster angle reads
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Test I2C initialization
    info!("I2C initialized successfully");

    // Enable motor driver
    enable.set_high().unwrap();

    info!("Running");
    let mut current_angle: f32 = 0.0;
    let mut debug_counter: u32 = 0;
    let mut loop_counter: u32 = 0;
    let mut last_debug_time: u64 = 0;

    // ADC smoothing state (exponential moving average)
    let mut filtered_vel_adc: f32 = 0.0;
    let mut filtered_trim_adc: f32 = 0.0;
    let mut adc_initialized = false;
    const ADC_ALPHA: f32 = 0.02; // Low-pass filter coefficient (lower = smoother)

    // Velocity ramp rate limit (rad/s per second)
    let mut ramped_velocity: f32 = 0.0;
    const VELOCITY_RAMP_RATE: f32 = 20.0; // max change of 20 rad/s per second
    let mut ramp_timestamp: u64 = 0;

    // Closed-loop velocity control
    loop {
        let now_us = timer.get_counter().ticks();
        loop_counter += 1;

        // Read potentiometers via ADC (12-bit: 0-4095) with smoothing
        let raw_vel: u16 = adc.read(&mut adc_pin_0).unwrap();
        let raw_trim: u16 = adc.read(&mut adc_pin_1).unwrap();

        // Initialize filter on first read, then apply exponential moving average
        if !adc_initialized {
            filtered_vel_adc = raw_vel as f32;
            filtered_trim_adc = raw_trim as f32;
            adc_initialized = true;
            ramp_timestamp = now_us;
        } else {
            filtered_vel_adc += ADC_ALPHA * (raw_vel as f32 - filtered_vel_adc);
            filtered_trim_adc += ADC_ALPHA * (raw_trim as f32 - filtered_trim_adc);
        }

        // Pot 0: target velocity 0..50 rad/s (from filtered ADC)
        let target_velocity: f32 = (filtered_vel_adc / 4095.0) * 50.0;

        // Pot 1: V/f trim factor (center = 1.0, range 0.9..1.1)
        let vf_trim: f32 = 0.9 + (filtered_trim_adc / 4095.0) * 0.2;

        // Ramp rate limit: smoothly transition to target velocity
        let ramp_dt_us = now_us.wrapping_sub(ramp_timestamp);
        let ramp_dt = ramp_dt_us as f32 * 1e-6;
        if ramp_dt > 0.0 && ramp_dt < 0.5 {
            let max_change = VELOCITY_RAMP_RATE * ramp_dt;
            let vel_diff = target_velocity - ramped_velocity;
            if vel_diff > max_change {
                ramped_velocity += max_change;
            } else if vel_diff < -max_change {
                ramped_velocity -= max_change;
            } else {
                ramped_velocity = target_velocity;
            }
        }
        ramp_timestamp = now_us;

        // Compute voltage limit from V/f curve using ramped velocity × trim
        motor.voltage_limit = (velocity_to_voltage(ramped_velocity) * vf_trim).clamp(0.0, 1.0);

        // Read encoder every loop
        // match read_angle(&mut i2c) {
        //     Ok(angle) => {
        //         current_angle = angle;
        //     }
        //     Err(_) => {}
        // }

        // Read current sensors

        // Run closed-loop velocity control
        //motor.velocity_closedloop(ramped_velocity, current_angle, now_us);
        motor.velocity_openloop(ramped_velocity, now_us);

        // Debug output every ~1000 loops
        debug_counter += 1;
        if debug_counter >= 1000 {
            debug_counter = 0;

            let elapsed_us = now_us.wrapping_sub(last_debug_time);
            let elapsed_s = elapsed_us as f32 * 1e-6;

            let loop_rate_hz = (loop_counter as f32) / elapsed_s;
            info!(
                "pot: {}, ramp: {}, vlim: {}, trim: {}, hz: {}",
                target_velocity, ramped_velocity, motor.voltage_limit, vf_trim, loop_rate_hz
            );
            loop_counter = 0;
            last_debug_time = now_us;
        }
    }
}

fn read_angle<T: I2c>(i2c: &mut T) -> Result<f32, T::Error> {
    let mut buf = [0u8; 2];
    i2c.write_read(0x36u8, &[0x0E], &mut buf)?;
    let angle_u16 = ((buf[0] as u16) << 8) | (buf[1] as u16);
    Ok(angle_u16 as f32 / 4096.0 * 360.0)
}
