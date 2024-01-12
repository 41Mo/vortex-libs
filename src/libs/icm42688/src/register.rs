#[allow(dead_code)]
#[repr(u8)]
pub enum Register {
    WhoAmI = 0x75,
    FifoConfig = 0x16,
    FifoConfig1 = 0x5f,
    IntfConfig0 = 0x4c,
    IntfConfig1 = 0x4d,
    SignalPathReset = 0x4b,
    PwrMgmt0 = 0x4e,
    GyroConfig0 = 0x4f,
    AccelConfig0 = 0x50,
    GyroConfigStatic2 = 0x0B,
    GyroConfigStatic3 = 0x0C,
    GyroConfigStatic4 = 0x0D,
    GyroConfigStatic5 = 0x0E,
    AccelConfigStatic2 = 0x03,
    AccelConfigStatic3 = 0x04,
    AccelConfigStatic4 = 0x05,
    BankSel = 0x76,
    FifoCounth = 0x2e,
    FifoData = 0x30,
}

impl Register {
    /// Get register address
    pub fn addr(self) -> u8 {
        self as u8
    }
}
