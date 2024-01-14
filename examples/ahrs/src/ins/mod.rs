#![allow(dead_code)]

pub struct INS {
    imu: ringbuf::StaticConsumer<'static, (nalgebra::Vector3<f32>, nalgebra::Vector3<f32>), 5>,
    dcm: nalgebra::Matrix3<f32>,
    rad_corr_c: f32,
}

impl INS {
    pub fn new(
        rb: ringbuf::StaticConsumer<'static, (nalgebra::Vector3<f32>, nalgebra::Vector3<f32>), 5>,
    ) -> Self {
        Self {
            imu: rb,
            dcm: nalgebra::Matrix3::<f32>::from_diagonal_element(1.0),
            rad_corr_c: 1.0,
        }
    }

    pub fn get_rpy(&self) -> (f32, f32, f32) {
        get_angles(&self.dcm)
    }

    pub fn update(&mut self) {
        if self.imu.len() == 0 {
            return;
        }
        let (acc, gyr) = self.imu.pop().unwrap();
        let dt = 0.01;
        let a_enu = self.dcm * acc;
        let mut w_enu = nalgebra::Vector3::<f32>::zeros();
        w_enu[0] = a_enu[1] * -self.rad_corr_c;
        w_enu[1] = a_enu[0] * self.rad_corr_c;
        let w_enu_hat = nalgebra::Matrix3::<f32>::new(
            0.0, -w_enu[2], w_enu[1], w_enu[2], 0.0, -w_enu[0], -w_enu[1], w_enu[0], 0.0,
        );
        let w_body_hat = nalgebra::Matrix3::<f32>::new(
            0.0, -gyr[2], gyr[1], gyr[2], 0.0, -gyr[0], -gyr[1], gyr[0], 0.0,
        );
        self.dcm += (self.dcm * w_body_hat - w_enu_hat * self.dcm) * dt;
    }
}

fn get_angles(dcm: &nalgebra::Matrix3<f32>) -> (f32, f32, f32) {
    use micromath::F32Ext;
    let c0 = F32Ext::sqrt(F32Ext::powf(dcm.m31, 2.0) + F32Ext::powf(dcm.m33, 2.0));
    let pitch = F32Ext::atan(dcm.m32 / c0);
    let roll = -F32Ext::atan2(dcm.m31, dcm.m33);
    let yaw = F32Ext::atan2(dcm.m12, dcm.m22);

    (roll, pitch, yaw)
}

#[allow(dead_code)]
const TO_DEG: f32 = 180.0 / 3.1415926;

#[allow(dead_code)]
fn to_deg(mut a_rad: (f32, f32, f32)) -> (f32, f32, f32) {
    a_rad.0 *= TO_DEG;
    a_rad.1 *= TO_DEG;
    a_rad.2 *= TO_DEG;
    a_rad
}
