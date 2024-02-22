use singleton::{FnNew, Singleton};

#[derive(Singleton)]
pub struct INS {
    imu: imu::Consumer,
    dcm: nalgebra::Matrix3<f32>,
    rad_corr_c: f32,
    is_anles_updated: bool,
    roll: Ftype,
    pitch: Ftype,
    yaw: Ftype,
}

type Ftype = f32;
type Matrix3F = nalgebra::Matrix3<Ftype>;
type Vector3F = nalgebra::Vector3<Ftype>;

impl singleton::FnNew for INS {
    fn new() -> Self {
        Self {
            imu: imu::get_imu_consumer(imu::ImuInstance::IMU1).unwrap(),
            dcm: Matrix3F::from_diagonal_element(1.0),
            rad_corr_c: 1.0,
            is_anles_updated: false,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        }
    }
}

impl INS {
    pub fn get_rpy(&mut self) -> (f32, f32, f32) {
        self.attitude_update();
        (self.roll, self.pitch, self.yaw)
    }

    #[allow(unused)]
    pub fn roll(&mut self) -> f32 {
        self.attitude_update();
        self.roll
    }

    #[allow(unused)]
    pub fn pitch(&mut self) -> f32 {
        self.attitude_update();
        self.pitch
    }

    #[allow(unused)]
    pub fn yaw(&mut self) -> f32 {
        self.attitude_update();
        self.yaw
    }

    fn update(&mut self) {
        let (acc, gyr, dt) = match self.imu.pop() {
            Some(v) => v,
            None => return,
        };
        let dt = (dt as f32)/1000.0;

        let a_enu = self.dcm * acc;
        let mut w_enu = Vector3F::zeros();
        w_enu[0] = a_enu[1] * -self.rad_corr_c;
        w_enu[1] = a_enu[0] * self.rad_corr_c;
        let w_enu_hat = Matrix3F::new(
            0.0, -w_enu[2], w_enu[1], w_enu[2], 0.0, -w_enu[0], -w_enu[1], w_enu[0], 0.0,
        );
        let w_body_hat = Matrix3F::new(
            0.0, -gyr[2], gyr[1], gyr[2], 0.0, -gyr[0], -gyr[1], gyr[0], 0.0,
        );
        self.is_anles_updated = false;
        self.dcm += (self.dcm * w_body_hat - w_enu_hat * self.dcm) * dt;
        self.attitude_update();
        let mut lacc = [0.0; 3];
        let mut lgyr = [0.0; 3];
        lacc.copy_from_slice(acc.as_slice());
        lgyr.copy_from_slice(gyr.as_slice());
        logger::log_msg(logger::LogMessages::LogINS(logger::INS {
            sample_time: embassy_time::Instant::now().as_micros(),
            att: [self.roll, self.pitch, self.yaw],
            acc: lacc,
            gyr: lgyr,
        }));
    }

    fn attitude_update(&mut self) {
        if self.is_anles_updated {
            return;
        }

        let (r, p, y) = get_angles(&self.dcm);
        self.roll = r;
        self.pitch = p;
        self.yaw = y;
        self.is_anles_updated = true;
    }
}

pub fn update() {
    INS::get().update()
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
