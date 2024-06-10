extern crate quaternion;
extern crate linalg;
extern crate utils;
extern crate libm;

use crate::gyroscope::{GyroscopeConfig, Gyroscope};
use crate::accelerometer::{AccelerometerConfig, Accelerometer};
use crate::magnetometer::{MagnetometerConfig, Magnetometer};
use crate::common::{OpenAHRSError, calculate_omega_matrix};

use quaternion::quaternion::Quaternion;
use linalg::matrix::Matrix;
use linalg::vector::Vector;
use linalg::common::EPSILON;

/*
pub const MARG: u8 = 1; // This mode uses the gyroscope, the accelerometer and the magnetometer.
pub const AM:   u8 = 2; // This mode uses only the accelerometer and the magnetometer.
*/

#[derive(Debug)]
#[derive(PartialEq)]
pub enum Mode {
    MARG = 1,
    AM = 2,
}

/*
impl Mode {
    fn is_valid_mode(value: u8) -> Option<Mode> {
        match value {
            1 => Some(Mode::MARG),
            2 => Some(Mode::AM),
            _ => None,
        }
    }
}
*/

#[derive(Debug)]
pub struct AQUA {
    gyr: Gyroscope,
    acc: Accelerometer,
    mag: Magnetometer,

    orientation: Quaternion,

    ts: f32,                    // Sampling period.
    adaptive: bool,             // Activate or not the adaptive gain.
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
    // This function is used to create a new AQUA filter.
    pub fn new() -> Result<Self, OpenAHRSError> {
        let aqua = Self {
            // Filter sensors (gyroscope (optionnal), accelerometer and magnetometer (used to perform gyro drift correction)).
            gyr: Gyroscope::new()?,
            acc: Accelerometer::new()?,
            mag: Magnetometer::new()?,

            orientation: Quaternion::new()?,    // Estimated attitude by the filter.

            // Filter settings.
            ts:         0.01_f32,               // Default sampling period.
            adaptive:   true,                   // Adaptive gain is activated by default.
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

    pub fn init(self: &mut Self,
        //mode: u8,
        mode: Mode,                                                         // Mode of the filter.
        qw: Option<f32>, qx: Option<f32>, qy: Option<f32>, qz: Option<f32>, // Initial attitude (optionnal).
        gyroscope_config: GyroscopeConfig,                                  // Gyroscope configuration.
        accelerometer_config: AccelerometerConfig,                          // Accelerometer configuration.
        magnetometer_config: MagnetometerConfig,                            // Magnetometer configuration.

    ) -> Result<(), OpenAHRSError> {
        /*
        let valid_mode = match Mode::is_valid_mode(mode) {
            Some(valid_mode) => valid_mode,
            None => return Err(LinalgError::InvalidAQUAMode),
        };

        match valid_mode {
            Mode::MARG => {
                // Initialize the gyroscope.
                // Initialize the accelerometer.
                // Initialize the magnetometer.
            }

            Mode::AM => {
                // Initialize the accelerometer.
                // Initialize the magnetometer.
            }
        }
        */

        if mode == Mode::MARG {
            self.gyr.init(gyroscope_config)?;       // Initialize the gyroscope.
            self.acc.init(accelerometer_config)?;   // Initialize the accelerometer.
            self.mag.init(magnetometer_config)?;    // Initialize the magnetometer.
        } else if mode == Mode::AM {
            self.acc.init(accelerometer_config)?;   // Initialize the accelerometer.
            self.mag.init(magnetometer_config)?;    // Initialize the magnetometer.
        } else {
            return Err(OpenAHRSError::InvalidAQUAMode);
        }

        // Check if all quaternion coordinates are present or absent.
        let quat = match (qw, qx, qy, qz) {
            (Some(qw), Some(qx), Some(qy), Some(qz)) => Some((qw, qx, qy, qz)),
            (None, None, None, None) => None,
            _ => return Err(OpenAHRSError::InvalidQuaternion),
        };

        // Set initial orientation manually.
        if let Some((qw, qx, qy, qz)) = quat {
            self.orientation.fill(qw, qx, qy, qz)?;
        } else {    // Automatically initialize orientation.
            // Add some code here.
        }

        Ok(())
    }

    pub fn update(self: &mut Self,
        gx: Option<f32>, gy: Option<f32>, gz: Option<f32>,  // Gyroscope raw measurements.
        ax: f32, ay: f32, az: f32,                          // Accelerometer raw measurements.
        mx: f32, my: f32, mz: f32                           // Magnetometer raw measurements.
    ) -> Result<(), OpenAHRSError> {
        // Check that the AQUA filter is initialized.
        if !self.initialized {
            // The AQUA filter is not initialized.
            Err(OpenAHRSError::AQUAFilterNotInit)   // Return an error.
        } else {
            // Flags to check if sensor measurements are valid.
            let mut gyr_ok: bool = false;
            let mut acc_ok: bool = false;
            let mut mag_ok: bool = false;

            // Check the type of mode.
            if self.mode == Mode::MARG {
                // Check if all gyroscope measurement are present or absent.
                let gyro_raw_measurements = match (gx, gy, gz) {
                    (Some(gx), Some(gy), Some(gz)) => Some((gx, gy, gz)),
                    (None, None, None) => None,
                    _ => return Err(OpenAHRSError::InvalidGyroRawMeasurements),
                };

                if let Some((gx, gy, gz)) = gyro_raw_measurements {
                    self.gyr.update(gx, gy, gz)?;   // Update the gyroscope with raw measurements to correct them.
                } else {    // Attempt to use MARG mode without providing raw gyro readings.
                    //return Err(OpenAHRSError::);  // Return an error.
                }
            }

            self.acc.update(ax, ay, az)?;   // Update the accelerometer with raw measurements to correct them.
            self.mag.update(mx, my, mz)?;   // Update the magnetometer with raw measurements to correct them.
















            // Retrieve corrected gyroscope measurements.
            let p = self.gyr.get_x_angular_rate()?;
            let q = self.gyr.get_y_angular_rate()?;
            let r = self.gyr.get_z_angular_rate()?;

            let mut w: Vector<f32> = Vector::new(); // Create the angular velocity pseudovector.
            w.init(3)?;                             // Initialize it.

            // Fill it with gyro-corrected measurements.
            w.set_element(0, p)?;
            w.set_element(1, q)?;
            w.set_element(2, r)?;

            // Normalize it.
            w.normalize()?;

            if w.calculate_norm()? > EPSILON {
                gyr_ok = true;
            }


            /*
            // Retrieve corrected accelerometer measurements.
            let ax = self.acc.get_x_acceleration()?;
            let ay = self.acc.get_y_acceleration()?;
            let az = self.acc.get_z_acceleration()?;

            // Retrieve corrected magnetometer measurements.
            let mx = self.mag.get_x_magnetic_field()?;
            let my = self.mag.get_y_magnetic_field()?;
            let mz = self.mag.get_z_magnetic_field()?;
            */


            if gyr_ok && acc_ok && mag_ok {
                // Prediction step.
                let mut omega = calculate_omega_matrix(p, q, r)?;   // Calculate the transformation matrix Ω(ω).
            }










            Ok(())  // Return no error.
        }
    }
}
