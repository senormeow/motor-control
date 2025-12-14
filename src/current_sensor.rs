// Bring the embedded-hal 0.2 OneShot trait into scope (rp2040-hal expects this).
use cortex_m::prelude::_embedded_hal_adc_OneShot;
use defmt::*;
use defmt_rtt as _;
use embedded_hal_0_2::adc::Channel;
use rp_pico::hal::adc::Adc;
use rp_pico::hal::adc::AdcPin;

pub struct CurrentSensor<P0: rp_pico::hal::gpio::AnyPin, P1: rp_pico::hal::gpio::AnyPin> {
    start_time: u64,
    end_time: u64,
    a0: u16,
    a1: u16,
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
            start_time: 0,
            end_time: 0,
            a0: 0,
            a1: 0,
            adc,
            adc_pin_0,
            adc_pin_1,
        }
    }

    pub fn read(&mut self) {
        self.a0 = self.adc.read(&mut self.adc_pin_0).unwrap();
        self.a1 = self.adc.read(&mut self.adc_pin_1).unwrap();
    }

    pub fn display(&mut self) {
        info!("Adc0: {}, Adc1: {}", self.a0, self.a1);
    }
}
