extern crate quaternion;
extern crate linalg;
extern crate utils;
extern crate libm;

use crate::gyroscope::Gyroscope;
use crate::accelerometer::Accelerometer;
use crate::magnetometer::Magnetometer;
use quaternion::quaternion::Quaternion;
use crate::common::OpenAHRSError;

/*
pub const MARG: u8 = 1; // This mode uses the gyroscope, the accelerometer and the magnetometer.
pub const AM:   u8 = 2; // This mode uses only the accelerometer and the magnetometer.
*/

#[derive(Debug)]
pub enum Mode {
    MARG = 1, // This mode uses the gyroscope, the accelerometer and the magnetometer.
    AM = 2,   // This mode uses only the accelerometer and the magnetometer.
}

impl Mode {
    fn is_valid_mode(value: u8) -> Option<Mode> {
        match value {
            1 => Some(Mode::MARG),
            2 => Some(Mode::AM),
            _ => None,
        }
    }
}

#[derive(Debug)]
pub struct AQUA {
    gyr: Gyroscope,
    acc: Accelerometer,
    mag: Magnetometer,

    orientation: Quaternion,

    ts: f32,
    adaptive: bool,
    alpha: f32,
    beta: f32,
    t1: f32,
    t2: f32,
    t_acc: f32,
    t_mag: f32,
    mode: Mode,
    order: u8,
    initialized: bool,
}

impl AQUA {
    pub fn new() -> Result<Self, OpenAHRSError> {
        let aqua = Self {
            // Filter sensors.
            gyr: Gyroscope::new()?,
            acc: Accelerometer::new()?,
            mag: Magnetometer::new()?,

            orientation: Quaternion::new()?,    // Estimated attitude by the filter.

            // Filter settings.
            ts:         0.01_f32,               // Sampling period.
            adaptive:   true,                   // Activate or not the adaptive gain.
            alpha:      0.01_f32,               // ...
            beta:       0.01_f32,               // ...
            t1:         0.1_f32,                // Adaptative gain first treshold.
            t2:         0.2_f32,                // Adaptative gain second treshold.
            t_acc:      0.9_f32,                // Interpolation treshold for the partial orientation (quaternion) determined from accelerometer measurements.
            t_mag:      0.9_f32,                // Interpolation treshold for the partial orientation (quaternion) determined from magnetometer measurements.
            //mode:       MARG,                   // Mode of the filter.
            mode:       Mode::MARG,             // Mode of the filter.
            order:      2_u8,                   // Order of the numerical integration method.
            //method:     Method::TAYLOR_SERIES,  // Numerical integration method.

            initialized: false,             // Initialization status.
        };

        Ok(aqua)    // Return the structure with no error.
    }

    /*
    pub fn init(
        // Some code here.
    ) -> Result<(), OpenAHRSError> {
        // Some code here.

        Ok(())
    }
    */
}
