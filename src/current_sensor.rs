// Bring the embedded-hal 0.2 OneShot trait into scope (rp2040-hal expects this).
use cortex_m::prelude::_embedded_hal_adc_OneShot;
use embedded_hal_0_2::adc::Channel;
use libm::sqrtf;
use rp_pico::hal::adc::Adc;
use rp_pico::hal::adc::AdcPin;

pub struct CurrentSensor<P0: rp_pico::hal::gpio::AnyPin, P1: rp_pico::hal::gpio::AnyPin> {
    a0: u16,
    a1: u16,
    // RMS accumulation
    sum_sq_a: f32,
    sum_sq_b: f32,
    sample_count: u32,
    // Last computed RMS values
    rms_a: f32,
    rms_b: f32,
    adc: Adc,
    adc_pin_0: AdcPin<P0>,
    adc_pin_1: AdcPin<P1>,
}

impl<P0, P1> CurrentSensor<P0, P1>
where
    P0: rp_pico::hal::gpio::AnyPin,
    P1: rp_pico::hal::gpio::AnyPin,
    AdcPin<P0>: Channel<Adc, ID = u8>,
    AdcPin<P1>: Channel<Adc, ID = u8>,
{
    pub fn new(adc: Adc, adc_pin_0: AdcPin<P0>, adc_pin_1: AdcPin<P1>) -> Self {
        Self {
            a0: 0,
            a1: 0,
            sum_sq_a: 0.0,
            sum_sq_b: 0.0,
            sample_count: 0,
            rms_a: 0.0,
            rms_b: 0.0,
            adc,
            adc_pin_0,
            adc_pin_1,
        }
    }

    /// Read current sensors and accumulate for RMS calculation
    pub fn read(&mut self) {
        self.a0 = self.adc.read(&mut self.adc_pin_0).unwrap();
        self.a1 = self.adc.read(&mut self.adc_pin_1).unwrap();

        // Convert to current and accumulate squared values
        let current_a = self.adc_to_current(self.a0);
        let current_b = self.adc_to_current(self.a1);

        self.sum_sq_a += current_a * current_a;
        self.sum_sq_b += current_b * current_b;
        self.sample_count += 1;
    }

    /// Convert ADC reading to current in Amps
    /// Assumes typical inline current sensor with 1.65V offset and ~0.1V/A sensitivity
    fn adc_to_current(&self, adc_value: u16) -> f32 {
        // ADC is 12-bit (0-4095) with 3.3V reference
        let voltage = (adc_value as f32 / 4095.0) * 3.3;
        // Offset is typically Vcc/2 = 1.65V, sensitivity ~0.1V/A
        // Adjust based on your sensor (e.g., ACS712, INA169, etc.)
        (voltage - 1.65) / 0.1
    }

    /// Calculate and reset RMS values
    /// Call this periodically (e.g., every 1000 samples)
    pub fn calculate_rms(&mut self) {
        if self.sample_count > 0 {
            self.rms_a = sqrtf(self.sum_sq_a / self.sample_count as f32);
            self.rms_b = sqrtf(self.sum_sq_b / self.sample_count as f32);
        }
        // Reset accumulators
        self.sum_sq_a = 0.0;
        self.sum_sq_b = 0.0;
        self.sample_count = 0;
    }

    /// Get RMS current for phase A (call calculate_rms first)
    pub fn get_rms_a(&self) -> f32 {
        self.rms_a
    }

    /// Get RMS current for phase B (call calculate_rms first)
    pub fn get_rms_b(&self) -> f32 {
        self.rms_b
    }

    /// Get instantaneous current for phase A
    pub fn get_current_a(&self) -> f32 {
        self.adc_to_current(self.a0)
    }

    /// Get instantaneous current for phase B
    pub fn get_current_b(&self) -> f32 {
        self.adc_to_current(self.a1)
    }
}
