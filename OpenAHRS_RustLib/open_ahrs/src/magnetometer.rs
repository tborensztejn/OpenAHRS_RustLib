extern crate linalg;

use linalg::matrix::{Matrix, copy_from};
use linalg::vector::Vector;
use linalg::linalg::{vector_to_matrix, get_col};
use crate::common::OpenAHRSError;

// Magnetometer configuration.
pub struct MagnetometerConfig {
    // Scaling correction factors.
    pub x_axis_scale_correction: f32,
    pub y_axis_scale_correction: f32,
    pub z_axis_scale_correction: f32,

    // Axes misalignment, non-orthogonality and soft-iron effects correction factors.
    pub xy_soft_iron_correction: f32,
    pub xz_soft_iron_correction: f32,
    pub yx_soft_iron_correction: f32,
    pub yz_soft_iron_correction: f32,
    pub zx_soft_iron_correction: f32,
    pub zy_soft_iron_correction: f32,

    // Hard-iron effects correction factors (static biases).
    pub x_axis_hard_iron_bias: f32,
    pub y_axis_hard_iron_bias: f32,
    pub z_axis_hard_iron_bias: f32,
}

// Default magnetometer configuration.
impl Default for MagnetometerConfig {
    // This function is used to generate a default magnetometer configuration.
    fn default() -> Self {
        Self {
            // Default scaling correction factors.
            x_axis_scale_correction: 1.0,
            y_axis_scale_correction: 1.0,
            z_axis_scale_correction: 1.0,

            // Default axes misalignment, non-orthogonality and soft-iron effects correction factors.
            xy_soft_iron_correction: 0.0,
            xz_soft_iron_correction: 0.0,
            yx_soft_iron_correction: 0.0,
            yz_soft_iron_correction: 0.0,
            zx_soft_iron_correction: 0.0,
            zy_soft_iron_correction: 0.0,

            // Default hard-iron effects correction factors (static biases).
            x_axis_hard_iron_bias: 0.0,
            y_axis_hard_iron_bias: 0.0,
            z_axis_hard_iron_bias: 0.0,
        }
    }
}

#[derive(Debug)]
pub struct Magnetometer {
    /* Ellipsoid Fitting Calibration settings. */
    /*
        Soft-iron effects:
        Description: these distortions depend on the orientation of the sensor in relation to magnetic fields or
        external forces.
        Cause: they are caused by inhomogeneous ferromagnetic materials around the sensor. These materials distort
        the magnetic field according to their own magnetic permeability, creating anisotropic distortion.
        Consequence: instead of a perfect sphere, the sensor's raw measurements form an ellipsoid, where the axes
        are stretched or compressed depending on the direction of distortion.

        Hard-iron effects:
        Description: these are constant shifts in the sensor's measurements, independent of the sensor's orientation.
        Cause: they are caused by permanent magnetic fields present around the sensor. For example, permanent magnets,
        magnetised metal parts or nearby electronic components can create a fixed magnetic field.
        Consequence: this constant shift moves the centre of the measurement sphere from the origin to a new fixed
        position.
    */

    scale_and_soft_iron_correction: Matrix, // Matrix which used to correct measurement distorsions (soft-iron effects, scale factors, axes misalignments and non-orthogonality).
    hard_iron_biases: Vector<f32>,          // Vector which contains static bias (hard-iron effects) of each axis of the three-dimensional magnetometer that should be remove from raw measurements.
    raw_measurements: Vector<f32>,          // Vector which contains raw measurements of the three-dimensional magnetometer.
    corrected_measurements: Vector<f32>,    // Vector which contains corrected measurements of the three-dimensional magnetometer.
    //conversion_factor: f32,

    initialized: bool,                      // Sensor initialisation flag.
}

impl Magnetometer {
    // This function is used to create a new magnetometer.
    pub fn new() -> Result<Self, OpenAHRSError> {
        let mut mag = Self {
            scale_and_soft_iron_correction: Matrix::new(),  // ...
            hard_iron_biases: Vector::new(),                // ...
            raw_measurements: Vector::new(),                // Create the vector containing the magnetic field intensity raw measurement of each of the magnetometer axes.
            corrected_measurements: Vector::new(),          // Create the vector containing the magnetic field intensity corrected measurement of each of the magnetometer axes.

            initialized: false, // Set initialisation flag to false (by default, the magnetometer is not initialised).
        };

        // Default correction matrix (identity matrix).
        mag.scale_and_soft_iron_correction.init(3, 3)?;
        mag.scale_and_soft_iron_correction.fill_identity()?;

        // Default static biases (zero biases).
        mag.hard_iron_biases.init(3)?;
        mag.hard_iron_biases.fill(0.0)?;

        // Default raw measurements (none).
        mag.raw_measurements.init(3)?;
        mag.raw_measurements.fill(0.0)?;

        // Default corrected measurements (none).
        mag.corrected_measurements.init(3)?;
        mag.corrected_measurements.fill(0.0)?;

        Ok(mag) // Return the structure with no error.
    }

    // This function is used to initialize a magnetometer.
    pub fn init(
        self: &mut Self, config: MagnetometerConfig) -> Result<(), OpenAHRSError> {
            // Check if the magnetometer has already been initialized.
            if self.initialized {
                // The magnetometer has already been configured.
                Err(OpenAHRSError::MagAlreadyInit)
            } else {    // Apply the configuration to the magnetometer.
                self.scale_and_soft_iron_correction.set_element(0, 0, config.x_axis_scale_correction)?;
                self.scale_and_soft_iron_correction.set_element(1, 1, config.y_axis_scale_correction)?;
                self.scale_and_soft_iron_correction.set_element(2, 2, config.z_axis_scale_correction)?;

                self.scale_and_soft_iron_correction.set_element(0, 1, config.xy_soft_iron_correction)?;
                self.scale_and_soft_iron_correction.set_element(1, 0, config.xz_soft_iron_correction)?;
                self.scale_and_soft_iron_correction.set_element(0, 2, config.yx_soft_iron_correction)?;
                self.scale_and_soft_iron_correction.set_element(2, 0, config.yz_soft_iron_correction)?;
                self.scale_and_soft_iron_correction.set_element(1, 2, config.zx_soft_iron_correction)?;
                self.scale_and_soft_iron_correction.set_element(2, 1, config.zy_soft_iron_correction)?;

                self.hard_iron_biases.set_element(0, config.x_axis_hard_iron_bias)?;
                self.hard_iron_biases.set_element(1, config.y_axis_hard_iron_bias)?;
                self.hard_iron_biases.set_element(2, config.z_axis_hard_iron_bias)?;

                self.initialized = true;    // Set the initialization flag to true.

                Ok(())  // Return no error.
            }
    }

    // This function is used to correct the raw measurements.
    fn correct(self: &mut Self) -> Result<(), OpenAHRSError> {
        self.corrected_measurements.sub(&self.raw_measurements, &self.hard_iron_biases)?;                           // Remove static biases (hard-iron effects) from raw measurements.
        let mut corrected_measurements = vector_to_matrix(&self.corrected_measurements)?;                           // Convert this vector into a matrix to perform matrix operations.
        corrected_measurements.mul(&self.scale_and_soft_iron_correction, &copy_from(&corrected_measurements)?)?;    // Correct measurement distorsions (soft-iron effects, scale factors, axes misalignments and non-orthogonality).
        get_col(&corrected_measurements, &mut self.corrected_measurements, 0)?;                                     // Perform matrix-to-vector conversion to store the corrected measurements.

        Ok(())  // Return no error.
    }

    // This function is used to update magnetometer raw measurements and correct them.
    pub fn update(self: &mut Self, ax: f32, ay: f32, az: f32) -> Result<(), OpenAHRSError> {
        // Check that the magnetometer is configured.
        if !self.initialized {
            // The magnetometer is not initialized.
            Err(OpenAHRSError::MagNotInit)  // Retrurn an error.
        } else {
            // Store the magnetometer raw measurements.
            self.raw_measurements.set_element(0, ax)?;
            self.raw_measurements.set_element(1, ay)?;
            self.raw_measurements.set_element(2, az)?;

            self.correct()?;    // Correct raw measurements.

            Ok(())  // Return no error.
        }
    }

    // This function is used to obtain the corrected magnetic field intensity measurement following the X axis.
    pub fn get_x_magnetic_field(self: &Self) -> Result<f32, OpenAHRSError> {
        // Check that the magnetometer is configured.
        if !self.initialized {
            // The magnetometer is not initialized.
            Err(OpenAHRSError::MagNotInit)  // Return an error.
        } else {
            let mx = self.corrected_measurements.get_element(0)?;   // Retrieve the value.

            Ok(mx)  // Return mx value with no error.
        }
    }

    // This function is used to obtain the corrected magnetic field intensity measurement following the Y axis.
    pub fn get_y_magnetic_field(self: &Self) -> Result<f32, OpenAHRSError> {
        // Check that the magnetometer is configured.
        if !self.initialized {
            // The magnetometer is not initialized.
            Err(OpenAHRSError::MagNotInit)  // Return an error.
        } else {
            let my = self.corrected_measurements.get_element(1)?;   // Retrieve the value.

            Ok(my)  // Return my value with no error.
        }
    }

    // This function is used to obtain the corrected magnetic field intensity measurement following the Y axis.
    pub fn get_z_magnetic_field(self: &Self) -> Result<f32, OpenAHRSError> {
        // Check that the magnetometer is configured.
        if !self.initialized {
            // The magnetometer is not initialized.
            Err(OpenAHRSError::MagNotInit)  // Return an error.
        } else {
            let mz = self.corrected_measurements.get_element(2)?;   // Retrieve the value.

            Ok(mz)  // Return mz value with no error.
        }
    }
}
