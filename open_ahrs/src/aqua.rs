extern crate quaternion;
extern crate linalg;
extern crate utils;
extern crate libm;

use crate::gyrometer::{GyrometerConfig, Gyrometer};
use crate::accelerometer::{AccelerometerConfig, Accelerometer};
use crate::magnetometer::{MagnetometerConfig, Magnetometer};
//use crate::common::{OpenAHRSError, NumericalIntegrationMethod as NIM, calculate_omega_matrix};
use crate::common::{OpenAHRSError, calculate_omega_matrix};

use quaternion::quaternion::Quaternion;
use linalg::matrix::Matrix;
use linalg::vector::Vector;
use linalg::common::EPSILON;
use utils::utils::in_range;
use libm::{sqrtf, fabsf};

#[derive(Debug)]
#[derive(PartialEq)]
pub enum Mode {
    MARG,   // This mode uses the gyrometer, the accelerometer and the magnetometer.
    AM,     // This mode uses only the accelerometer and the magnetometer.
}

#[derive(Debug)]
pub struct Aqua {
    mode: Mode,             // Mode of the filter.

    // Filter sensors (gyrometer (optional), accelerometer and magnetometer (used to perform gyro drift correction)).
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
    g: f32,                 // Earth gravitational acceleration magnitude.
    h: f32,                 // Earth magnetic field magnitude.
    //order: u8,              // Order of the numerical integration method.
    //method: NIM,            // Numerical integration method.

    initialized: bool,      // Initialization status.
}

impl Aqua {
    /// This method is used to create a new Aqua filter.
    pub fn new() -> Result<Self, OpenAHRSError> {
        let aqua = Self {
            mode:       Mode::MARG,         // Default mode of the filter.

            // Filter sensors (gyrometer (optional), accelerometer and magnetometer (used to perform gyro drift correction)).
            gyr: Gyrometer::new()?,
            acc: Accelerometer::new()?,
            mag: Magnetometer::new()?,

            attitude: Vector::new(),        // Estimated attitude by the filter as a quaternion.

            // Filter settings.
            ts:         0.01,               // Default sampling period.
            adaptive:   true,               // Default adaptive gain is activated by default.
            alpha:      0.01,               // Default interpolation parameter for the SLERP and LERP algorithm for the accelerometer.
            beta:       0.01,               // Default interpolation parameter for the SLERP and LERP algorithm for the magnetometer.
            t1:         0.1,                // Default adaptative gain first treshold.
            t2:         0.2,                // Default adaptative gain second treshold.
            t_acc:      0.9,                // Default interpolation treshold for the partial attitude quaternion determined from accelerometer measurements.
            t_mag:      0.9,                // Default interpolation treshold for the partial attitude quaternion determined from magnetometer measurements.
            g:          9.81,               // Default Earth gravitational acceleration magnitude.
            h:          48.0,               // Default Earth magnetic field magnitude.
            //order:      2,                  // Default order of the numerical integration method.
            //method:     NIM::ClosedForm,    // Default numerical integration method.

            initialized: false,             // Default initialization status.
        };

        Ok(aqua)    // Return the new filter with no error.
    }

    // This function is used to initialize the Aqua filter.
    pub fn init(self: &mut Self,
        mode: Mode,                                                         // Define the mode of the filter.
        qw: Option<f32>, qx: Option<f32>, qy: Option<f32>, qz: Option<f32>, // Define the initial attitude (optional).
        gyrometer_config: Option<&GyrometerConfig>,                         // Define the gyrometer configuration (optional).
        accelerometer_config: &AccelerometerConfig,                         // Define the accelerometer configuration.
        magnetometer_config: &MagnetometerConfig,                           // Define the magnetometer configuration.
        ts: f32,                                                            // Define the sampling period.
        adaptive: bool,                                                     // Define if the interpolation parameter is adaptive or fixed.
        alpha: f32,                                                         // Define the (initial) interpolation parameter for the SLERP and LERP algorithm for the accelerometer.
        beta: f32,                                                          // Define the (initial) interpolation parameter for the SLERP and LERP algorithm for the magnetometer.
        t1: f32,                                                            // Define the first treshold used to calculate adaptative interpolation parameter.
        t2: f32,                                                            // Define the second treshold used to calculate adaptative interpolation parameter. The second treshold must be strictly greater than the first one.
        t_acc: f32,                                                         // Define the interpolation treshold for the SLERP and LERP algorithm for the partial attitude quaternion determined from accelerometer measurements.
        t_mag: f32,                                                         // Define the interpolation treshold for the SLERP and LERP algorithm for the partial attitude quaternion determined from magnetometer measurements.
        g: Option<f32>,                                                     // Define Earth gravitational acceleration magnitude (optional).
        h: Option<f32>                                                      // Define Earth magnetic field magnitude (optional).
    ) -> Result<(), OpenAHRSError> {
        // Check if the filter has already been initialized.
        if self.initialized {
            // The filter has already been initialized.
            return Err(OpenAHRSError::AquaFilterAlreadyInit); // Return an error.
        }

        self.attitude.init(4)?;         // Initialize the attitude quaternion.
        self.attitude.fill_identity()?; // Fill it as identity quaternion.

        // Check the mode.
        if mode == Mode::MARG {
            // Check if the gyrometer configuration is present or absent.
            let config = match gyrometer_config {
                Some(config) => config,                             // The gyrometer configuration is present.
                None => return Err(OpenAHRSError::NoGyroconfig),    // No gyrometer configuration and it's required. Return an error.
            };

            self.gyr.init(&config)?;                 // Initialize the gyrometer.
            self.acc.init(&accelerometer_config)?;   // Initialize the accelerometer.
            self.mag.init(&magnetometer_config)?;    // Initialize the magnetometer.
        } else if mode == Mode::AM {
            self.acc.init(&accelerometer_config)?;   // Initialize the accelerometer.
            self.mag.init(&magnetometer_config)?;    // Initialize the magnetometer.
        } else {
            // Invalid mode.
            return Err(OpenAHRSError::InvalidAquaMode); // Return an error.
        }

        // Check if all quaternion coordinates are present or absent.
        let quat = match (qw, qx, qy, qz) {
            (Some(qw), Some(qx), Some(qy), Some(qz)) => Some((qw, qx, qy, qz)), // All components supplied, initial quaternion present.
            (None, None, None, None) => None,                                   // No components supplied, no initial quaternion.
            _ => return Err(OpenAHRSError::InvalidInitQuaternion),              // The quaternion is not valid (some components were not supplied). Return an error.
        };

        // Set initial attitude manually.
        if let Some((qw, qx, qy, qz)) = quat {
            self.attitude.fillq(qw, qx, qy, qz)?;
        } else {    // Automatically initialize attitude.
            // Add some code here.
            // In MARG mode, the initial quaternion is determined from the accelerometer and magnetometer in the same way as in AM mode.
        }

        self.mode = mode;               // Set the mode.

        self.ts = ts;                   // Set sampling rate.
        self.adaptive = adaptive;       // Set ...
        self.alpha = alpha;             // Set ...
        self.beta = beta;               // Set ...
        self.t1 = t1;                   // Set ...
        self.t2 = t2;                   // Set ...
        self.t_acc = t_acc;             // Set ...
        self.t_mag = t_mag;             // Set ...
        self.g = g.unwrap_or(self.g);   // Set Earth gravitational acceleration magnitude.
        self.h = h.unwrap_or(self.h);   // Set Earth magnetic field magnitude.
        //self.method = method;           // Set the numerical integration method that will be used to estimate the attitude.
        //self.order = order;             // Set the order of the method.

        // Add some code here.

        self.initialized = true;        // Set initialization status flag to true.

        Ok(())  // Return no error.
    }

    // This function sets the accelerometer cnnfiguration
    pub fn set_accelerometer_config(&mut self, config: &AccelerometerConfig) -> Result<(), OpenAHRSError> {
        self.acc.set_config(&config)?;   // Configure the accelerometer.
        Ok(())
    }

    // This function sets the gyrometer cnnfiguration
    pub fn set_gyrometer_config(&mut self, config: &GyrometerConfig) -> Result<(), OpenAHRSError> {
        self.gyr.set_config(&config)?;   // Configure the gyrometer.
        Ok(())
    }

    // This function sets the magnetometer cnnfiguration
    pub fn set_magnetometer_config(&mut self, config: &MagnetometerConfig) -> Result<(), OpenAHRSError> {
        self.mag.set_config(&config)?;   // Configure the magnetometer.
        Ok(())
    }

    // This function gets the accelerometer cnnfiguration
    pub fn get_accelerometer_config(&mut self) -> Result<AccelerometerConfig, OpenAHRSError> {
        let config = self.acc.get_config()?;   // Get the accelerometer configuration.
        Ok(config)
    }

    // This function gets the gyrometer cnnfiguration
    pub fn get_gyrometer_config(&mut self) -> Result<GyrometerConfig, OpenAHRSError> {
        let config = self.gyr.get_config()?;   // Get the gyrometer configuration.
        Ok(config)
    }

    // This function gets the magnetometer cnnfiguration
    pub fn get_magnetometer_config(&mut self) -> Result<MagnetometerConfig, OpenAHRSError> {
        let config = self.mag.get_config()?;   // Get the magnetometer configuration.
        Ok(config)
    }

    // This function is used to perform adaptive quaternion interpolation based on LERP and SLERP.
    fn interpolate(quat: &Vector<f32>, alpha: f32, treshold: f32) -> Result<Vector<f32>, OpenAHRSError> {
        // Check that the interpolation parameter is valid (it must be between 0 and 1 inclusive).
        if !in_range(treshold, 0.0_f32, 1.0_f32) {
            // The interpolation parameter is not valid.
            return Err(OpenAHRSError::InvalidAquaInterpolationTreshold);    // Return an error.
        }

        let mut idendity_quat: Vector<f32> = Vector::new();     // Create the identity quaternion.
        idendity_quat.init(4)?;                                 // Initialize it.
        idendity_quat.fill_identity()?;                         // Fill it.

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
            return Err(OpenAHRSError::InvalidAdaptiveGainTresholds) // Return an error.
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

    // This function is used to update the Aqua filter.
    pub fn update(self: &mut Self,
        gx: Option<f32>, gy: Option<f32>, gz: Option<f32>,  // Gyrometer raw measurements.
        ax: f32, ay: f32, az: f32,                          // Accelerometer raw measurements.
        mx: f32, my: f32, mz: f32                           // Magnetometer raw measurements.
    ) -> Result<(), OpenAHRSError> {
        // Check that the Aqua filter is initialized.
        if !self.initialized {
            // The Aqua filter is not initialized.
            return Err(OpenAHRSError::AquaFilterNotInit);   // Return an error.
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
                return Err(OpenAHRSError::NoGyroRawMeasurements);   // Return an error.
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

        let a_local_norm = a_local.calculate_norm()?;   // Calculate the norm of the acceleration vector.

        // Retrieve acceleration vector components and normalize them (we want normalized components without normalizing the vector).
        if a_local_norm > EPSILON {
            ax = a_local.get_element(0)? / a_local_norm;
            ay = a_local.get_element(1)? / a_local_norm;
            az = a_local.get_element(2)? / a_local_norm;
        }

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

        //let m_local_norm = m_local.calculate_norm()?;   // Calculate the norm of the magnetic field intensity vector.

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
            omega.mul_by_scalar_in_place(0.5)?;
            let derivative_quat = omega.mul_new(&self.attitude.convert_to_matrix()?)?;

            let mut delta_quat = derivative_quat.col_to_vector(0)?;
            delta_quat.mul_by_scalar_in_place(self.ts)?;

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
            // Predicted gravity (Source: Roberto G. Valenti, Ivan Dryanovski and Jizhong Xiao. Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs. New York: Sensors, 2015, p. 16, eq. 44).
            let gx = a_global.get_element(0)?;
            let gy = a_global.get_element(1)?;
            let gz = a_global.get_element(2)?;

            // Determine the quaternion (partial orientation) from the accelerometer measurements.
            // Source: Roberto G. Valenti, Ivan Dryanovski and Jizhong Xiao. Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs. New York: Sensors, 2015, p. 16, eq. 47.
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
                self.alpha = self.calculate_adaptative_gain(self.alpha, &a_local, self.g)?;
            }

            let acc_quat_interpolated = Self::interpolate(&acc_quat, self.alpha, self.t_acc)?;

            // Source: Roberto G. Valenti, Ivan Dryanovski and Jizhong Xiao. Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs. New York: Sensors, 2015, p. 17, eq. 53.
            self.attitude.mul(&gyr_quat, &acc_quat_interpolated)?;

            // Magnetometer-based correction step.
            let mut dcm_local_to_global = self.attitude.convert_to_dcm()?;
            dcm_local_to_global.transpose()?;
            temp.mul(&dcm_local_to_global, &m_local.convert_to_matrix()?)?;
            // Global frame magnetic vector (Source: Roberto G. Valenti, Ivan Dryanovski and Jizhong Xiao. Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs. New York: Sensors, 2015, p. 18, eq. 54).
            let m_global = temp.col_to_vector(0)?;

            let lx = m_global.get_element(0)?;
            let ly = m_global.get_element(1)?;

            // Source: Roberto G. Valenti, Ivan Dryanovski and Jizhong Xiao. Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs. New York: Sensors, 2015, p. 10, eq. 28.
            let gamma = lx*lx + ly*ly;

            /*
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
            */

            qw = sqrtf(gamma + lx * sqrtf(gamma)) / sqrtf(2.0 * gamma);
            qx = 0.0;
            qy = 0.0;
            qz = ly / (sqrtf(2.0) * sqrtf(gamma + lx * sqrtf(gamma)));

            mag_quat.fillq(qw, qx, qy, qz)?;    // Fill it.

            /*
            if self.adaptive {
                m_local.mul_by_scalar_in_place(m_local_norm)?;
                self.beta =  self.calculate_adaptative_gain(self.alpha, &m_local, self.h)?;
                m_local.mul_by_scalar_in_place(1.0 / m_local_norm)?;
            }
            */

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

    pub fn get_attitude(self: &Self) -> Result<Vector<f32>, OpenAHRSError> {
        // Check if the filter has already been initialized.
        if !self.initialized {
            // The filter has already been initialized.
            return Err(OpenAHRSError::AquaFilterNotInit); // Return an error.
        }

        Ok(self.attitude.duplicate()?)  // Return estimated attitude with no error.
    }
}
