extern crate linalg;

use linalg::matrix::{Matrix, copy_from};
use linalg::vector::Vector;
use linalg::linalg::{vector_to_matrix, get_col};
use crate::common::OpenAHRSError;

#[derive(Debug)]
pub struct Gyroscope {
    /* Axis misalignment and non-orthogonality correction matrix. */
    /*
        Scale correction:
        Three-dimensional gyrometers can have variations in sensitivity (scale) between the different axes. This means that for the same angular velocity,
        one axis may give a different measurement from another. The diagonal elements of the correction matrix can be used to correct these potential deviations.

        Axis non-orthogonality correction:
        The various axes of the three-dimensional gyrometer may not be completely orthogonal. This introduces errors where rotation around one axis influences
        measurements on the other axes. The off-diagonal elements of the correction matrix help to correct these potential deviations.
    */
    axes_misalignment_correction: Matrix,
    static_biases: Vector<f64>,             // Vector which contains static bias of each axis of the three-dimensional gyrometer that should be remove from raw measurements.
    raw_measurements: Vector<f64>,          // Vector which contains raw measurements of the three-dimensional gyroscope.
    corrected_measurements: Vector<f64>,    // Vector which contains corrected measurements of the three-dimensional gyroscope.
    initialized: bool,  // Sensor initialisation flag.
}

impl Gyroscope {
    // This function is used to create a new gyroscope.
    pub fn new() -> Result<Self, OpenAHRSError> {
        let mut gyr = Self {
            axes_misalignment_correction: Matrix::new(),
            static_biases: Vector::new(),
            raw_measurements: Vector::new(),
            corrected_measurements: Vector::new(),
            initialized: false,
        };

        // Default correction matrix (identity matrix).
        gyr.axes_misalignment_correction.init(3, 3)?;
        gyr.axes_misalignment_correction.fill_identity()?;

        // Default static biases (zero biases).
        gyr.static_biases.init(3)?;
        gyr.static_biases.fill(0.0)?;

        // Default raw measurements (none).
        gyr.raw_measurements.init(3)?;
        gyr.raw_measurements.fill(0.0)?;

        // Default corrected measurements (none).
        gyr.corrected_measurements.init(3)?;
        gyr.corrected_measurements.fill(0.0)?;

        Ok(gyr)
    }

    // This function is used to initialize a gyroscope.
    pub fn init(
        self: &mut Self,
        x_axis_scaling_correction_factor: f64,              //
        y_axis_scaling_correction_factor: f64,              //
        z_axis_scaling_correction_factor: f64,              //
        xy_axes_non_orthogonality_correction_factor: f64,   //
        xz_axes_non_orthogonality_correction_factor: f64,   //
        yx_axes_non_orthogonality_correction_factor: f64,   //
        yz_axes_non_orthogonality_correction_factor: f64,   //
        zx_axes_non_orthogonality_correction_factor: f64,   //
        zy_axes_non_orthogonality_correction_factor: f64,   //
        x_axis_static_bias: f64,                            // Static bias on X axis measurements.
        y_axis_static_bias: f64,                            // Static bias on Y axis measurements.
        z_axis_static_bias: f64                             // Static bias on Z axis measurements.
        ) -> Result<(), OpenAHRSError> {
            // Check if the matrix has already been initialized.
            if self.initialized {
                // The gyroscope has already been configured.
                Err(OpenAHRSError::GyrAlreadyInit)
            } else {
                // Set the scaling correction factors.
                self.axes_misalignment_correction.set_element(0, 0, x_axis_scaling_correction_factor)?;
                self.axes_misalignment_correction.set_element(1, 1, y_axis_scaling_correction_factor)?;
                self.axes_misalignment_correction.set_element(2, 2, z_axis_scaling_correction_factor)?;

                // Set the cross-axes non-orthogonality correction factors.
                self.axes_misalignment_correction.set_element(1, 0, xy_axes_non_orthogonality_correction_factor)?;
                self.axes_misalignment_correction.set_element(2, 0, xz_axes_non_orthogonality_correction_factor)?;
                self.axes_misalignment_correction.set_element(0, 1, yx_axes_non_orthogonality_correction_factor)?;
                self.axes_misalignment_correction.set_element(2, 1, yz_axes_non_orthogonality_correction_factor)?;
                self.axes_misalignment_correction.set_element(0, 2, zx_axes_non_orthogonality_correction_factor)?;
                self.axes_misalignment_correction.set_element(1, 2, zy_axes_non_orthogonality_correction_factor)?;

                // Set the axes static biases.
                self.static_biases.set_element(0, x_axis_static_bias)?;
                self.static_biases.set_element(1, y_axis_static_bias)?;
                self.static_biases.set_element(2, z_axis_static_bias)?;

                self.initialized = true;    // Set the initialization flag to true.

                Ok(())  // Return no error.
            }
    }

    // This function is used to correct the raw measurements.
    fn correct(self: &mut Self) -> Result<(), OpenAHRSError> {
        self.corrected_measurements.sub(&self.raw_measurements, &self.static_biases)?;                          // Remove static biases from raw measurements.
        let mut corrected_measurements: Matrix = vector_to_matrix(&self.corrected_measurements)?;               // Convert this vector into a matrix to perform matrix operations.
        corrected_measurements.mul(&self.axes_misalignment_correction, &copy_from(&corrected_measurements)?)?;  // Correct axes non-orthonormality.
        get_col(&corrected_measurements, &mut self.corrected_measurements, 0)?;                                 // Perform matrix-to-vector conversion to store the corrected measurements.

        Ok(())  // Return no error.
    }

    // This function is used to update gyroscope raw measurements anc correct them.
    pub fn update(self: &mut Self, gx: f64, gy: f64, gz: f64) -> Result<(), OpenAHRSError> {
        // Check that the gyroscope is configured.
        if !self.initialized {
            // The gyroscope is not initialized.
            Err(OpenAHRSError::GyrNotInit)
        } else {
            // Store the gyroscope raw measurements.
            self.raw_measurements.set_element(0, gx)?;
            self.raw_measurements.set_element(1, gy)?;
            self.raw_measurements.set_element(2, gz)?;

            self.correct()?;    // Correct raw measurements.

            Ok(())
        }
    }

    // This function is used to obtain the corrected angulaur rate measurement arround the X axis.
    pub fn get_x_angular_rate(self: &Self) -> Result<f64, OpenAHRSError> {
        // Check that the gyroscope is configured.
        if !self.initialized {
            // The gyroscope is not initialized.
            Err(OpenAHRSError::GyrNotInit)
        } else {
            Ok(self.corrected_measurements.get_element(0)?) // Return p value.
        }
    }

    // This function is used to obtain the corrected angulaur rate measurement arround the Y axis.
    pub fn get_y_angular_rate(self: &Self) -> Result<f64, OpenAHRSError> {
        // Check that the gyroscope is configured.
        if !self.initialized {
            // The gyroscope is not initialized.
            Err(OpenAHRSError::GyrNotInit)
        } else {
            Ok(self.corrected_measurements.get_element(1)?) // Return q value.
        }
    }

    // This function is used to obtain the corrected angulaur rate measurement arround the Z axis.
    pub fn get_z_angular_rate(self: &Self) -> Result<f64, OpenAHRSError> {
        // Check that the gyroscope is configured.
        if !self.initialized {
            // The gyroscope is not initialized.
            Err(OpenAHRSError::GyrNotInit)
        } else {
            Ok(self.corrected_measurements.get_element(2)?) // Return r value.
        }
    }
}
