use defmt::error;
use embedded_hal::digital::OutputPin;
use oxidrone_hal::platform::PlatformAbstraction;

pub struct FlightController<P>
where
    P: PlatformAbstraction,
{
    state: bool,
    primary_led: P::OutputPin,
    count: usize,
}

impl<P> FlightController<P>
where
    P: PlatformAbstraction,
    P::OutputPin: embedded_hal::digital::OutputPin,
{
    pub fn new(mut platform: P) -> Self {
        let primary_led = platform.primary_led().unwrap();
        Self {
            state: false,
            primary_led,
            count: 0,
        }
    }

    pub fn update(&mut self) {
        if self.state {
            error!("heartbeat {=usize}", self.count);
            self.primary_led.set_low().unwrap();
            self.count += 1;
        } else {
            self.primary_led.set_high().unwrap();
        }
        self.state = !self.state;
    }
}
