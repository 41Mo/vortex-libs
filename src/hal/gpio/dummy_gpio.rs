pub struct DummyGpioOut();

pub struct DummyError;
impl OutputPin for DummyGpioOut {
    type Error = DummyError;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
