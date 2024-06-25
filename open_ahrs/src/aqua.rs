extern crate quaternion;
extern crate linalg;
extern crate utils;
extern crate libm;

use crate::gyrometer::{GyrometerConfig, Gyrometer};
use crate::accelerometer::{AccelerometerConfig, Accelerometer};
use crate::magnetometer::{MagnetometerConfig, Magnetometer};
use crate::common::{OpenAHRSError, NumericalIntegrationMethod as NIM, calculate_omega_matrix};

use quaternion::quaternion::Quaternion;
use linalg::matrix::Matrix;
use linalg::vector::Vector;
use utils::utils::in_range;
use libm::{sqrtf, fabsf};

#[derive(Debug)]
#[derive(PartialEq)]
pub enum Mode {
    MARG,   // This mode uses the gyrometer, the accelerometer and the magnetometer.
    AM,     // This mode uses only the accelerometer and the magnetometer.
}

#[derive(Debug)]
pub struct AQUA {
    // Filter sensors (gyrometer (optionnal), accelerometer and magnetometer (used to perform gyro drift correction)).
    gyr: Gyrometer,
    acc: Accelerometer,
    mag: Magnetometer,

    attitude: Vector<f32>,  // Estimated attitude by the filter.

    ts: f32,                // Sampling period.
    adaptive: bool,         // Activate or not the adaptive gain.
    alpha: f32,             // Interpolation parameter for the SLERP and LERP algorithm for the accelerometer.
    beta: f32,              // Interpolation parameter for the SLERP and LERP algorithm for the magnetometer.
    t1: f32,                // Adaptative gain first treshold.
    t2: f32,                // Adaptative gain second treshold.
    t_acc: f32,             // Interpolation treshold for the partial attitude quaternion determined from accelerometer measurements.
    t_mag: f32,             // Interpolation treshold for the partial attitude quaternion determined from magnetometer measurements.
    mode: Mode,             // Mode of the filter.
    order: u8,              // Order of the numerical integration method.
    method: NIM,            // Numerical integration method.

    initialized: bool,      // Initialization status.
}

impl AQUA {
    /// This method is used to create a new AQUA filter.
    pub fn new() -> Result<Self, OpenAHRSError> {
        let aqua = Self {
            // Filter sensors (gyrometer (optionnal), accelerometer and magnetometer (used to perform gyro drift correction)).
            gyr: Gyrometer::new()?,         // ...
            acc: Accelerometer::new()?,     // ...
            mag: Magnetometer::new()?,      // ...

            attitude: Vector::new(),        // Estimated attitude by the filter as a quaternion.

            // Filter settings.
            ts:         0.01,               // Default sampling period.
            adaptive:   true,               // Default adaptive gain is activated by default.
            alpha:      0.01_f32,           // Default interpolation parameter for the SLERP and LERP algorithm for the accelerometer.
            beta:       0.01_f32,           // Default interpolation parameter for the SLERP and LERP algorithm for the magnetometer.
            t1:         0.1_f32,            // Default adaptative gain first treshold.
            t2:         0.2_f32,            // Default adaptative gain second treshold.
            t_acc:      0.9_f32,            // Default interpolation treshold for the partial attitude quaternion determined from accelerometer measurements.
            t_mag:      0.9_f32,            // Default interpolation treshold for the partial attitude quaternion determined from magnetometer measurements.
            mode:       Mode::MARG,         // Default mode of the filter.
            order:      2_u8,               // Default order of the numerical integration method.
            method:     NIM::ClosedForm,    // Default numerical integration method.

            initialized: false,             // Default initialization status.
        };

        Ok(aqua)    // Return the new filter with no error.
    }

    // This function is used to initialize the AQUA filter.
    pub fn init(self: &mut Self,
        //mode: u8,
        mode: Mode,                                                         // Define the mode of the filter.
        qw: Option<f32>, qx: Option<f32>, qy: Option<f32>, qz: Option<f32>, // Define the initial attitude (optionnal).
        gyrometer_config: GyrometerConfig,                                  // Define the gyrometer configuration.
        accelerometer_config: AccelerometerConfig,                          // Define the accelerometer configuration.
        magnetometer_config: MagnetometerConfig,                            // Define the magnetometer configuration.
        ts: f32,                                                            // Define the sampling period.
        adaptive: bool,                                                     // Define if the interpolation parameter is adaptive or fixed.
        alpha: f32,                                                         // Define the (initial) interpolation parameter for the SLERP and LERP algorithm for the accelerometer.
        beta: f32,                                                          // Define the (initial) interpolation parameter for the SLERP and LERP algorithm for the magnetometer.
        t1: f32,                                                            // Define the first treshold used to calculate adaptative interpolation parameter.
        t2: f32,                                                            // Define the second treshold used to calculate adaptative interpolation parameter. The second treshold must be strictly greater than the first one.
        t_acc: f32,                                                         // Define the interpolation treshold for the SLERP and LERP algorithm for the partial attitude quaternion determined from accelerometer measurements.
        t_mag: f32,                                                         // Define the interpolation treshold for the SLERP and LERP algorithm for the partial attitude quaternion determined from magnetometer measurements.
    ) -> Result<(), OpenAHRSError> {
        // Check if the filter has already been initialized.
        if self.initialized {
            // The filter has already been initialized.
            return Err(OpenAHRSError::AQUAFilterAlreadyInit); // Return an error.
        }

        self.attitude.init(4)?;                     // Initialize the attitude quaternion.

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
            self.attitude.fillq(qw, qx, qy, qz)?;
        } else {    // Automatically initialize attitude.
            // Add some code here.
        }

        self.ts = ts;               // Set sampling rate.
        self.adaptive = adaptive;   // ...
        self.alpha = alpha;         // Set
        self.beta = beta;
        self.t1 = t1;
        self.t2 = t2;
        self.t_acc = t_acc;
        self.t_mag = t_mag;
        //self.method = method;       // Set the numerical integration method that will be used to estimate the attitude.
        //self.order = order;         // Set the order of the method.

        // Add some code here.

        Ok(())  // Return no error.
    }

    // This function is used to perform adaptive quaternion interpolation based on LERP and SLERP.
    fn interpolate(quat: &Vector<f32>, alpha: f32, treshold: f32) -> Result<Vector<f32>, OpenAHRSError> {
        // Check that the interpolation parameter is valid (it must be between 0 and 1 inclusive).
        if !in_range(treshold, 0.0_f32, 1.0_f32) {
            // The interpolation parameter is not valid.
            return Err(OpenAHRSError::InvalidAQUAInterpolationTreshold) // Return an error.
        }

        let mut idendity_quat: Vector<f32> = Vector::new(); // Create the identity quaternion.
        idendity_quat.init(4)?;                             // Initialize it.
        idendity_quat.fill_identity()?;                     // Fill it.

        let mut interpolated_quat: Vector<f32> = Vector::new(); // Create the interpolated quaternion.
        interpolated_quat.init(4)?;                             // Initialize it.

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
    fn calculate_adaptative_gain(self: &Self, gain: f32, vect_global: &Vector<f32>, vect_norm_reference: f32) -> Result<f32, OpenAHRSError> {
        // Check if the second treshold is strictly greater than the first one.
        if self.t2 <= self.t1 {
            // The second treshold is not striclty greater than the first one.
            //return Err(OpenAHRSError::) // Return an error.
        }

        let vect_norm = vect_global.calculate_norm()?;
        let norm_error = fabsf((vect_norm - vect_norm_reference)/vect_norm_reference);

        let mut new_gain: f32 = 0.0;

        // Check if magnitude error falls within the specified thresholds.
        if self.t1 < norm_error && norm_error < self.t2 {
			new_gain = gain*(self.t1 - norm_error)/self.t1;
		}

        // If the magnitude error is below or equal to the first threshold, the gain doesn't change.
        if norm_error <= self.t1 {
            new_gain = gain;
        }

        Ok(new_gain)    // Return the new gain with no error.
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
            return Err(OpenAHRSError::AQUAFilterNotInit);   // Return an error.
        }


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

        // Retrieve corrected accelerometer measurements.
        let mut ax = self.acc.get_x_acceleration()?;
        let mut ay = self.acc.get_y_acceleration()?;
        let mut az = self.acc.get_z_acceleration()?;

        let mut a_local: Vector<f32> = Vector::new();   // Create the acceleration vector expressed in the local reference frame.
        a_local.init(3)?;                               // Initialize it.

        // Fill it with accelerometer corrected measurements.
        a_local.set_element(0, ax)?;
        a_local.set_element(1, ay)?;
        a_local.set_element(2, az)?;

        a_local.normalize()?;   // Normalize it.

        // Retrieve vector components.
        ax = a_local.get_element(0)?;
        ay = a_local.get_element(1)?;
        az = a_local.get_element(2)?;

        // Retrieve corrected magnetometer measurements.
        let mx = self.mag.get_x_magnetic_field()?;
        let my = self.mag.get_y_magnetic_field()?;
        let mz = self.mag.get_z_magnetic_field()?;

        let mut m_local: Vector<f32> = Vector::new();   // Create the magnetic field intensity vector expressed in the local reference frame.
        m_local.init(3)?;                               // Initialize it.

        // Fill it with magnetometer corrected measurements.
        m_local.set_element(0, mx)?;
        m_local.set_element(1, my)?;
        m_local.set_element(2, mz)?;

        m_local.normalize()?;   // Normalize it.

        // Define quaternion components.
        let mut qw: f32;
        let mut qx: f32;
        let mut qy: f32;
        let mut qz: f32;

        let mut acc_quat: Vector<f32> = Vector::new();  // Quaternion defining the orientation of the system and determined by accelerometer and gyrometer measurements.
        acc_quat.init(4)?;                              // Initialize it.

        let mut mag_quat: Vector<f32> = Vector::new();  // Quaternion defining the orientation of the system and determined by magnetometer and gyrometer measurements.
        mag_quat.init(4)?;                              // Initialize it.

        let mut temp = Matrix::new();
        temp.init(3, 1)?;

        // Check the type of mode.
        if self.mode == Mode::MARG {
            // Retrieve corrected gyrometer measurements.
            let p = self.gyr.get_x_angular_rate()?;
            let q = self.gyr.get_y_angular_rate()?;
            let r = self.gyr.get_z_angular_rate()?;

            let mut w_local: Vector<f32> = Vector::new();   // Create the angular velocity pseudovector expressed in the local reference frame.
            w_local.init(3)?;                               // Initialize it.

            // Fill it with gyrometer corrected measurements.
            w_local.set_element(0, p)?;
            w_local.set_element(1, q)?;
            w_local.set_element(2, r)?;

            // Prediction step.
            // Check whether it would not be more appropriate to use the AR filter to determine the quaternion with the 3-axis gyro.
            let mut omega = calculate_omega_matrix(p, q, r)?;   // Calculate the transformation matrix Ω(ω).
            omega.mul_by_scalar(0.5)?;
            let derivative_quat = omega.muln(&self.attitude.convert_to_matrix()?)?;


            let mut delta_quat = derivative_quat.col_to_vector(0)?;
            delta_quat.mul_by_scalar(self.ts)?;

            let mut gyr_quat: Vector<f32> = Vector::new();  // Quaternion defining the orientation of the system and determined by gyrometer measurements.
            gyr_quat.init(4)?;
            gyr_quat.add(&self.attitude, &delta_quat)?;
            gyr_quat.normalize()?;  // Normalize the quaternion to ensure is remains unitary.

            // Accelerometer-based correction step.
            let mut dcm_local_to_global = gyr_quat.convert_to_dcm()?;
            dcm_local_to_global.transpose()?;
            temp.mul(&dcm_local_to_global, &a_local.convert_to_matrix()?)?;
            let a_global = temp.col_to_vector(0)?;

            // Retrieve vector components.
            let gx = a_global.get_element(0)?;
            let gy = a_global.get_element(1)?;
            let gz = a_global.get_element(2)?;

            // Determine the quaternion (partial orientation) from the accelerometer measurements.
            if gz < 0.0 {
                let l1 = sqrtf((1.0 - gz) / 2.0);
                qw = l1;
                qx = gy / (2.0 * l1);
                qy = -gx / (2.0 * l1);
                qz = 0.0;
            } else {
                let l2 = sqrtf((1.0 + gz) / 2.0);
                qw = gy / (2.0 * l2);
                qx = l2;
                qy = 0.0;
                qz = -gx / (2.0 * l2);
            }

            acc_quat.fillq(qw, qx, qy, qz)?;    // Fill it.

            if self.adaptive {
                self.alpha = self.calculate_adaptative_gain(self.alpha, &a_global, -9.81)?;
            }

            let acc_quat_interpolated = Self::interpolate(&acc_quat, self.alpha, self.t_acc)?;

            self.attitude.mul(&gyr_quat, &acc_quat_interpolated)?;

            // Magnetometer-based correction step.
            let mut dcm_local_to_global = self.attitude.convert_to_dcm()?;
            dcm_local_to_global.transpose()?;
            temp.mul(&dcm_local_to_global, &m_local.convert_to_matrix()?)?;
            let m_global = temp.col_to_vector(0)?;

            let lx = m_global.get_element(0)?;
            let ly = m_global.get_element(1)?;

            let gamma = lx*lx + ly*ly;

            if lx >= 0.0 {
                qw = sqrtf(gamma + lx * sqrtf(gamma)) / sqrtf(2.0 * gamma);
                qx = 0.0;
                qy = 0.0;
                qz = ly / (sqrtf(2.0) * sqrtf(gamma + lx * sqrtf(gamma)));
            } else {
                qw = ly / (sqrtf(2.0) * sqrtf(gamma - lx * sqrtf(gamma)));
                qx = 0.0;
                qy = 0.0;
                qz = sqrtf(gamma - lx * sqrtf(gamma)) / sqrtf(2.0 * gamma);
            }

            mag_quat.fillq(qw, qx, qy, qz)?;    // Fill it.

            if self.adaptive {
                self.beta =  self.calculate_adaptative_gain(self.alpha, &m_global, 0.0)?;
            }

            let mag_quat_interpolated = Self::interpolate(&mag_quat, self.beta, self.t_mag)?;

            self.attitude.mul(&self.attitude.duplicate()?, &mag_quat_interpolated)?;
        } else if self.mode == Mode::AM {
            // Determine the partial orientation as a quaternion from the accelerometer measurements.
            if az < 0.0 {
                let l1 = sqrtf((1.0 - az) / 2.0);
                qw = l1;
                qx = ay / (2.0 * l1);
                qy = -ax / (2.0 * l1);
                qz = 0.0;
            } else {
                let l2 = sqrtf((1.0 + az) / 2.0);
                qw = ay / (2.0 * l2);
                qx = l2;
                qy = 0.0;
                qz = -ax / (2.0 * l2);
            }

            acc_quat.fillq(qw, qx, qy, qz)?;    // Fill it.

            // Determine the partial orientation in the form of a quaternion from the magnetometer measurements.
            let mut partial_dcm_local_to_global = acc_quat.convert_to_dcm()?;
            partial_dcm_local_to_global.transpose()?;
            temp.mul(&partial_dcm_local_to_global, &m_local.convert_to_matrix()?)?;
            let m_global = temp.col_to_vector(0)?;

            let lx = m_global.get_element(0)?;
            let ly = m_global.get_element(1)?;

            let gamma = lx*lx + ly*ly;

            if lx >= 0.0 {
                qw = sqrtf(gamma + lx * sqrtf(gamma)) / sqrtf(2.0 * gamma);
                qx = 0.0;
                qy = 0.0;
                qz = ly / (sqrtf(2.0) * sqrtf(gamma + lx * sqrtf(gamma)));
            } else {
                qw = ly / (sqrtf(2.0) * sqrtf(gamma - lx * sqrtf(gamma)));
                qx = 0.0;
                qy = 0.0;
                qz = sqrtf(gamma - lx * sqrtf(gamma)) / sqrtf(2.0 * gamma);
            }

            mag_quat.fillq(qw, qx, qy, qz)?;    // Fill it.

            self.attitude.mul(&acc_quat, &mag_quat)?;
        }

        self.attitude.normalize()?;

        Ok(())  // Return no error.
    }
}
