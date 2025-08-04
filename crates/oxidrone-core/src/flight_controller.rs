use embedded_hal::digital::OutputPin;
use oxidrone_hal::platform::PlatformAbstraction;

pub struct FlightController<P>
where
    P: PlatformAbstraction,
{
    state: bool,
    primary_led: P::OutputPin,
    secondary_led: P::OutputPin,
}

impl<P> FlightController<P>
where
    P: PlatformAbstraction,
    P::OutputPin: embedded_hal::digital::OutputPin,
{
    pub fn new(mut platform: P) -> Self {
        let primary_led = platform.primary_led().unwrap();
        let secondary_led = platform.secondary_led().unwrap();
        Self {
            state: false,
            primary_led,
            secondary_led,
        }
    }

    pub fn update(&mut self) {
        if self.state {
            self.primary_led.set_low().unwrap();
            self.secondary_led.set_high().unwrap();
        } else {
            self.primary_led.set_high().unwrap();
            self.secondary_led.set_low().unwrap();
        }
        self.state = !self.state;
    }
}
