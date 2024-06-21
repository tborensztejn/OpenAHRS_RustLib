extern crate linalg;

use linalg::matrix::Matrix;
use linalg::vector::Vector;
use crate::common::OpenAHRSError;

// Accelerometer configuration.
pub struct AccelerometerConfig {
    // Scaling correction factors.
    pub x_axis_scale_correction: f32,
    pub y_axis_scale_correction: f32,
    pub z_axis_scale_correction: f32,

    // Axes misalignment and non-orthogonality correction factors.
    pub xy_axes_misalignment_correction: f32,
    pub xz_axes_misalignment_correction: f32,
    pub yx_axes_misalignment_correction: f32,
    pub yz_axes_misalignment_correction: f32,
    pub zx_axes_misalignment_correction: f32,
    pub zy_axes_misalignment_correction: f32,

    // Static biases.
    pub x_axis_static_bias: f32,
    pub y_axis_static_bias: f32,
    pub z_axis_static_bias: f32
}

// Default accelerometer configuration.
impl Default for AccelerometerConfig {
    // This function is used to generate a default accelerometer configuration.
    fn default() -> Self {
        Self {
            // Default scaling correction factors.
            x_axis_scale_correction: 1.0,
            y_axis_scale_correction: 1.0,
            z_axis_scale_correction: 1.0,

            // Default axes misalignment and non-orthogonality correction factors.
            xy_axes_misalignment_correction: 0.0,
            xz_axes_misalignment_correction: 0.0,
            yx_axes_misalignment_correction: 0.0,
            yz_axes_misalignment_correction: 0.0,
            zx_axes_misalignment_correction: 0.0,
            zy_axes_misalignment_correction: 0.0,

            // Default static biases.
            x_axis_static_bias: 0.0,
            y_axis_static_bias: 0.0,
            z_axis_static_bias: 0.0,
        }
    }
}

#[derive(Debug)]
pub struct Accelerometer {
    /* Axis misalignment and non-orthogonality correction matrix. */
    /*
        Scale correction:
        Three-dimensional accelerometer can have variations in sensitivity (scale) between the different axes. This means that for the same acceleration,
        one axis may give a different measurement from another. The diagonal elements of the correction matrix can be used to correct these potential deviations.

        Axes misalignment and non-orthogonality correction:
        The various axes of the three-dimensional accelerometer may not be completely orthogonal. This introduces errors where rotation around one axis influences
        measurements on the other axes. The off-diagonal elements of the correction matrix help to correct these potential deviations.
    */

    scale_and_axes_misalignment_correction: Matrix, // Matrix used to correct axes misalignment and scale errors.
    static_biases: Vector<f32>,                     // Vector which contains static bias of each axis of the three-dimensional accelerometer that should be remove from raw measurements.
    raw_measurements: Vector<f32>,                  // Vector which contains raw measurements of the three-dimensional accelerometer.
    corrected_measurements: Vector<f32>,            // Vector which contains corrected measurements of the three-dimensional accelerometer.
    //conversion_factor: f32,

    initialized: bool,                      // Sensor initialization flag.
}

impl Accelerometer {
    // This function is used to create a new accelerometer.
    pub fn new() -> Result<Self, OpenAHRSError> {
        let mut acc = Self {
            scale_and_axes_misalignment_correction: Matrix::new(),  // Create the matrix that will be used to correct the scale error of each of the accelerometer axes, misalignment and non-orthogonality problems.
            static_biases: Vector::new(),                           // Create the vector containing the static bias of each of the accelerometer axes.
            raw_measurements: Vector::new(),                        // Create the vector containing the angular velocity raw measurement of each of the accelerometer axes.
            corrected_measurements: Vector::new(),                  // Create the vector containing the angular velocity corrected measurement of each of the accelerometer axes.

            initialized: false, // Set initialization flag to false (by default, the accelerometer is not initialized).
        };

        // Default correction matrix (identity matrix).
        acc.scale_and_axes_misalignment_correction.init(3, 3)?;
        acc.scale_and_axes_misalignment_correction.fill_identity()?;

        // Default static biases (zero biases).
        acc.static_biases.init(3)?;
        acc.static_biases.fill(0.0)?;

        // Default raw measurements (none).
        acc.raw_measurements.init(3)?;
        acc.raw_measurements.fill(0.0)?;

        // Default corrected measurements (none).
        acc.corrected_measurements.init(3)?;
        acc.corrected_measurements.fill(0.0)?;

        Ok(acc) // Return the new structure with no error.
    }

    // This function is used to initialize a accelerometer.
    pub fn init(self: &mut Self, config: AccelerometerConfig) -> Result<(), OpenAHRSError> {
        // Check if the accelerometer has already been initialized.
        if self.initialized {
            // The accelerometer has already been configured.
            Err(OpenAHRSError::AccAlreadyInit)  // Return an error.
        } else {    // Apply the configuration to the accelerometer.
            self.scale_and_axes_misalignment_correction.set_element(0, 0, config.x_axis_scale_correction)?;
            self.scale_and_axes_misalignment_correction.set_element(1, 1, config.y_axis_scale_correction)?;
            self.scale_and_axes_misalignment_correction.set_element(2, 2, config.z_axis_scale_correction)?;

            self.scale_and_axes_misalignment_correction.set_element(1, 0, config.xy_axes_misalignment_correction)?;
            self.scale_and_axes_misalignment_correction.set_element(2, 0, config.xz_axes_misalignment_correction)?;
            self.scale_and_axes_misalignment_correction.set_element(0, 1, config.yx_axes_misalignment_correction)?;
            self.scale_and_axes_misalignment_correction.set_element(2, 1, config.yz_axes_misalignment_correction)?;
            self.scale_and_axes_misalignment_correction.set_element(0, 2, config.zx_axes_misalignment_correction)?;
            self.scale_and_axes_misalignment_correction.set_element(1, 2, config.zy_axes_misalignment_correction)?;

            self.static_biases.set_element(0, config.x_axis_static_bias)?;
            self.static_biases.set_element(1, config.y_axis_static_bias)?;
            self.static_biases.set_element(2, config.z_axis_static_bias)?;

            self.initialized = true;    // Set the initialization flag to true.

            Ok(())  // Return no error.
        }
    }

    // This function is used to correct the raw measurements.
    fn correct(self: &mut Self) -> Result<(), OpenAHRSError> {
        self.corrected_measurements.sub(&self.raw_measurements, &self.static_biases)?;                                      // Remove static biases from raw measurements.
        let mut corrected_measurements = self.corrected_measurements.convert_to_matrix()?;                                  // Convert this vector into a matrix to perform matrix operations.
        corrected_measurements.mul(&self.scale_and_axes_misalignment_correction, &corrected_measurements.duplicate()?)?;    // Correct scale error and axes misalignment and non-orthonormality.
        corrected_measurements.get_col(&mut self.corrected_measurements, 0)?;                                               // Perform matrix-to-vector conversion to store the corrected measurements.

        Ok(())  // Return no error.
    }

    // This function is used to update accelerometer raw measurements and correct them.
    pub fn update(self: &mut Self, gx: f32, gy: f32, gz: f32) -> Result<(), OpenAHRSError> {
        // Check that the accelerometer is configured.
        if !self.initialized {
            // The accelerometer is not initialized.
            Err(OpenAHRSError::AccNotInit)  // Retrurn an error.
        } else {
            // Store the accelerometer raw measurements.
            self.raw_measurements.set_element(0, gx)?;
            self.raw_measurements.set_element(1, gy)?;
            self.raw_measurements.set_element(2, gz)?;

            self.correct()?;    // Correct raw measurements.

            Ok(())  // Return no error.
        }
    }

    // This function is used to obtain the corrected acceleration measurement following the X axis.
    pub fn get_x_acceleration(self: &Self) -> Result<f32, OpenAHRSError> {
        // Check that the accelerometer is configured.
        if !self.initialized {
            // The accelerometer is not initialized.
            Err(OpenAHRSError::AccNotInit)  // Return an error.
        } else {
            let ax = self.corrected_measurements.get_element(0)?;   // Retrieve the value.

            Ok(ax)  // Return ax value with no error.
        }
    }

    // This function is used to obtain the corrected acceleration measurement following the Y axis.
    pub fn get_y_acceleration(self: &Self) -> Result<f32, OpenAHRSError> {
        // Check that the accelerometer is configured.
        if !self.initialized {
            // The accelerometer is not initialized.
            Err(OpenAHRSError::AccNotInit)  // Return an error.
        } else {
            let ay = self.corrected_measurements.get_element(1)?;   // Retrieve the value.

            Ok(ay)  // Return ay value with no error.
        }
    }

    // This function is used to obtain the corrected acceleration measurement following the Z axis.
    pub fn get_z_acceleration(self: &Self) -> Result<f32, OpenAHRSError> {
        // Check that the accelerometer is configured.
        if !self.initialized {
            // The accelerometer is not initialized.
            Err(OpenAHRSError::AccNotInit)  // Return an error.
        } else {
            let az = self.corrected_measurements.get_element(2)?;   // Retrieve the value.

            Ok(az)  // Return az value with no error.
        }
    }
}
