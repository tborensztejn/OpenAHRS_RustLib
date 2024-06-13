extern crate quaternion;
extern crate linalg;
extern crate utils;
extern crate libm;

use crate::gyrometer::{GyrometerConfig, Gyrometer};
use crate::accelerometer::{AccelerometerConfig, Accelerometer};
use crate::magnetometer::{MagnetometerConfig, Magnetometer};
use crate::common::{OpenAHRSError, calculate_omega_matrix};

use quaternion::quaternion::{Quaternion, copy_from, vector_to_quaternion};
//use quaternion::quaternion::copy_from as copy_from_quaternion;
use linalg::linalg::{vector_to_matrix, col_to_vector};
use linalg::matrix::{Matrix, mul};
use linalg::vector::Vector;
use linalg::common::EPSILON;
use utils::utils::in_range;
use libm::sqrtf;

/*
pub const MARG: u8 = 1; // This mode uses the gyrometer, the accelerometer and the magnetometer.
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
    gyr: Gyrometer,
    acc: Accelerometer,
    mag: Magnetometer,

    attitude: Quaternion,

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
            // Filter sensors (gyrometer (optionnal), accelerometer and magnetometer (used to perform gyro drift correction)).
            gyr: Gyrometer::new()?,
            acc: Accelerometer::new()?,
            mag: Magnetometer::new()?,

            attitude: Quaternion::new()?,       // Estimated attitude by the filter.

            // Filter settings.
            ts:         0.01_f32,               // Default sampling period.
            adaptive:   true,                   // Adaptive gain is activated by default.
            alpha:      0.01_f32,               // ...
            beta:       0.01_f32,               // ...
            t1:         0.1_f32,                // Adaptative gain first treshold.
            t2:         0.2_f32,                // Adaptative gain second treshold.
            t_acc:      0.9_f32,                // Interpolation treshold for the partial attitude (quaternion) determined from accelerometer measurements.
            t_mag:      0.9_f32,                // Interpolation treshold for the partial attitude (quaternion) determined from magnetometer measurements.
            //mode:       MARG,                   // Mode of the filter.
            mode:       Mode::MARG,             // Mode of the filter.
            order:      2_u8,                   // Order of the numerical integration method.
            //method:     Method::TAYLOR_SERIES,  // Numerical integration method.

            initialized: false,             // Initialization status.
        };

        Ok(aqua)    // Return the structure with no error.
    }

    // This function is used to initialize the AQUA filter.
    pub fn init(self: &mut Self,
        //mode: u8,
        mode: Mode,                                                         // Mode of the filter.
        qw: Option<f32>, qx: Option<f32>, qy: Option<f32>, qz: Option<f32>, // Initial attitude (optionnal).
        gyrometer_config: GyrometerConfig,                                  // Gyrometer configuration.
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
                // Initialize the gyrometer.
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
            self.gyr.init(gyrometer_config)?;       // Initialize the gyrometer.
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

        // Set initial attitude manually.
        if let Some((qw, qx, qy, qz)) = quat {
            self.attitude.fill(qw, qx, qy, qz)?;
        } else {    // Automatically initialize attitude.
            // Add some code here.
        }

        Ok(())
    }

    // This function is used to perform adaptive quaternion interpolation based on LERP and SLERP.
    fn interpolate(quat: &Quaternion, alpha: f32, treshold: f32) -> Result<Quaternion, OpenAHRSError> {
        // Check that the interpolation parameter is valid (it must be between 0 and 1 inclusive).
        if !in_range(treshold, 0.0_f32, 1.0_f32) {
            // The interpolation parameter is not valid.
            return Err(OpenAHRSError::InvalidAQUAInterpolationTreshold) // Return an error.
        }

        let mut idendity_quat = Quaternion::new()?;     // Create the identity quaternion.
        idendity_quat.fill_identity()?;                 // Fill it.

        let mut interpolated_quat = Quaternion::new()?; // Create the interpolated quaternion.

        if quat.get_qw()? > treshold {  // Use the LERP algorithm because it's more computationally efficient.
            interpolated_quat.lerp(&idendity_quat, &quat, alpha)?;
            interpolated_quat.normalize()?; // Normalize the interpolated quaternion.
        } else {    // Use the SLERP algorithm (otherwise the approximation error would be too great).
            interpolated_quat.slerp(&idendity_quat, &quat, alpha)?;
        }

        //interpolated_quat.normalize()?; // Normalize the interpolated quaternion.

        Ok(interpolated_quat)   // Return interpolated quaternion with no error.
    }

    // This function is used to calculate adaptative gain.
    fn calculate_adaptative_gain() -> Result<(), OpenAHRSError> {
        Ok(())
    }

    // This function is used to update the AQUA filter.
    pub fn update(self: &mut Self,
        gx: Option<f32>, gy: Option<f32>, gz: Option<f32>,  // Gyrometer raw measurements.
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
                // Check if all gyrometer measurement are present or absent.
                let gyro_raw_measurements = match (gx, gy, gz) {
                    (Some(gx), Some(gy), Some(gz)) => Some((gx, gy, gz)),
                    (None, None, None) => None,
                    _ => return Err(OpenAHRSError::InvalidGyroRawMeasurements),
                };

                if let Some((gx, gy, gz)) = gyro_raw_measurements {
                    self.gyr.update(gx, gy, gz)?;   // Update the gyrometer with raw measurements to correct them.
                } else {    // Attempt to use MARG mode without providing raw gyro readings.
                    //return Err(OpenAHRSError::);  // Return an error.
                }
            }

            self.acc.update(ax, ay, az)?;   // Update the accelerometer with raw measurements to correct them.
            self.mag.update(mx, my, mz)?;   // Update the magnetometer with raw measurements to correct them.














            // Retrieve corrected gyrometer measurements.
            let mut p = self.gyr.get_x_angular_rate()?;
            let mut q = self.gyr.get_y_angular_rate()?;
            let mut r = self.gyr.get_z_angular_rate()?;

            let mut w: Vector<f32> = Vector::new(); // Create the angular velocity pseudovector.
            w.init(3)?;                             // Initialize it.

            // Fill it with gyrometer corrected measurements.
            w.set_element(0, p)?;
            w.set_element(1, q)?;
            w.set_element(2, r)?;

            // Check that the measurements are valid.
            if w.calculate_norm()? > EPSILON {
                gyr_ok = true;
            }

            // Retrieve corrected accelerometer measurements.
            let mut ax = self.acc.get_x_acceleration()?;
            let mut ay = self.acc.get_y_acceleration()?;
            let mut az = self.acc.get_z_acceleration()?;

            let mut a: Vector<f32> = Vector::new(); // Create the acceleration vector.
            a.init(3)?;                             // Initialize it.

            // Fill it with accelerometer corrected measurements.
            a.set_element(0, p)?;
            a.set_element(1, q)?;
            a.set_element(2, r)?;

            // Check that the measurements are valid.
            if a.calculate_norm()? > EPSILON {
                acc_ok = true;
            }

            a.normalize()?; // Normalize it.

            ax = a.get_element(0)?;
            ay = a.get_element(1)?;
            az = a.get_element(2)?;

            // Retrieve corrected magnetometer measurements.
            let mut mx = self.mag.get_x_magnetic_field()?;
            let mut my = self.mag.get_y_magnetic_field()?;
            let mut mz = self.mag.get_z_magnetic_field()?;

            let mut m: Vector<f32> = Vector::new(); // Create the magnetic field intensity vector.
            m.init(3)?;                             // Initialize it.

            // Fill it with magnetometer corrected measurements.
            m.set_element(0, p)?;
            m.set_element(1, q)?;
            m.set_element(2, r)?;

            // Check that the measurements are valid.
            if m.calculate_norm()? > EPSILON {
                mag_ok = true;
            }

            m.normalize()?; // Normalize it.

            mx = m.get_element(0)?;
            my = m.get_element(1)?;
            mz = m.get_element(2)?;


            if gyr_ok && acc_ok && mag_ok {
                // Prediction step.
                // Check whether it would not be more appropriate to use the AR filter to determine the quaternion with the 3-axis gyro.
                let mut omega = calculate_omega_matrix(p, q, r)?;   // Calculate the transformation matrix Ω(ω).
                omega.mul_by_scalar(0.5)?;
                let mut derivative_quat = mul(&omega, &vector_to_matrix(&self.attitude.get_vect()?)?)?;
                derivative_quat.mul_by_scalar(self.ts)?;

                /*
                let derivative_quat = col_to_vector(&derivative_quat, 0)?;
                let derivative_quat = vector_to_quaternion(&derivative_quat)?;
                */

                let mut gyr_quat = Quaternion::new()?;
                gyr_quat.add(&self.attitude, &vector_to_quaternion(&col_to_vector(&derivative_quat)?)?)?;
                //gyr_quat.add(&self.attitude, &derivative_quat);
                gyr_quat.normalize()?;  // Normalize the quaternion to ensure is remains unitary.









            } else if acc_ok && mag_ok {
                // Add some code here.
            }


























            Ok(())  // Return no error.
        }
    }
}
