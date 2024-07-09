extern crate quaternion;
extern crate linalg;
extern crate utils;
extern crate libm;

use crate::gyrometer::{GyrometerConfig, Gyrometer};
use crate::accelerometer::{AccelerometerConfig, Accelerometer};
use crate::magnetometer::{MagnetometerConfig, Magnetometer};
use crate::common::OpenAHRSError;

use quaternion::quaternion::Quaternion;
use linalg::matrix::Matrix;
use linalg::vector::Vector;
use linalg::common::EPSILON;
use utils::utils::in_range;
//use libm::{sqrtf, fabsf};

#[derive(Debug)]
pub struct Davenport {
    // Filter sensors (accelerometer and magnetometer).
    acc: Accelerometer,
    mag: Magnetometer,

    attitude: Vector<f32>,  // Estimated attitude by the filter.

    ts: f32,                // Sampling period.
    w1: f32,                // Accelerometer observations weight.
    w2: f32,                // Magnetometer observations weight.
    g: f32,                 // Earth gravitational acceleration magnitude.
    mdip: f32,              // Earth magnetic field dip.

    initialized: bool,      // Initialization status.
}

impl Davenport {
    /// This method is used to create a new Davenport's filter based on Q method.
    pub fn new() -> Result<Self, OpenAHRSError> {
        let davenport = Self {
            // Filter sensors (accelerometer and magnetometer).
            acc: Accelerometer::new()?,
            mag: Magnetometer::new()?,

            attitude: Vector::new(),    // Estimated attitude by the filter as a quaternion.

            // Filter settings.
            ts:         0.01,           // Default sampling period.
            w1:         1.0,            // Default accelerometer observations weight.
            w2:         1.0,            // Default magnetometer observations weight.
            g:          9.81,           // Default Earth gravitational acceleration magnitude.
            mdip:       0.0,            // Default Earth magnetic field dip.

            initialized: false,         // Default initialization status.
        };

        Ok(davenport)   // Return the new filter with no error.
    }

    /// This method is used to initialize the Davenport's filter based on Q method.
    pub fn init(self: &mut Self,
        accelerometer_config: AccelerometerConfig,  // Define the accelerometer configuration.
        magnetometer_config: MagnetometerConfig,    // Define the magnetometer configuration.
        ts: f32,                                    // Define the sampling period.
        w1: f32,                                    // Define accelerometer observations weight.
        w2: f32,                                    // Define magnetometer observations weight.
        g: Option<f32>,                             // Define Earth gravitational acceleration magnitude (optional).
        mdip: Option<f32>                           // Define Earth magnetic field dip. (optional).
    ) -> Result<(), OpenAHRSError> {
        // Check if the filter has already been initialized.
        if self.initialized {
            // The filter has already been initialized.
            return Err(OpenAHRSError::DavenportFilterAlreadyInit);  // Return an error.
        }

        self.attitude.init(4)?;         // Initialize the attitude quaternion.
        self.attitude.fill_identity()?; // Fill it as identity quaternion.

        // Check that the sensors observations weights are valid (they must be between 0 (not inclusive) and 1 inclusive).
        if !in_range(w1, EPSILON, 1.0_f32) || !in_range(w2, EPSILON, 1.0_f32){
            // The sensor(s) observation(s) weight(s) is/are not valid.
            return Err(OpenAHRSError::InvalidDavenportWeights); // Return an error.
        }

        self.w1 = w1;   // Set accelerometer observations weight.
        self.w2 = w2;   // Set magnetometer observations weight.

        self.g = g.unwrap_or(self.g);           // Set Earth gravitational acceleration magnitude.
        self.mdip = mdip.unwrap_or(self.mdip);  // Set Earth magnetic field magnitude.

        self.initialized = true;    // Set initialization status flag to true.

        Ok(())  // Return no error.
    }

    /// This method is used to update the Davenport's filter based on Q method.
    pub fn update(self: &mut Self,
        ax: f32, ay: f32, az: f32,  // Accelerometer raw measurements.
        mx: f32, my: f32, mz: f32   // Magnetometer raw measurements.
    ) -> Result<(), OpenAHRSError> {
        // Check that the Davenport's filter based on Q method is initialized.
        if !self.initialized {
            // The Davenport's filter based on Q method is not initialized.
            return Err(OpenAHRSError::DavenportFilterNotInit);   // Return an error.
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

        // Add some code here.

        Ok(())  // Return no error.
    }
}
