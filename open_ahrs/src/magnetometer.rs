extern crate linalg;

use linalg::matrix::Matrix;
use linalg::vector::Vector;
use linalg::linalg::{vector_to_matrix, get_col, set_col};
use crate::common::OpenAHRSError;

#[derive(Debug)]
pub struct Magnetometer {
    /* Ellipsoid Fitting Calibration settings. */
    /*
        Hard-iron effects:
        Description: these are constant shifts in the sensor's measurements, independent of the sensor's orientation.
        Cause: they are caused by permanent magnetic fields present around the sensor. For example, permanent magnets,
        magnetised metal parts or nearby electronic components can create a fixed magnetic field.
        Consequence: this constant shift moves the centre of the measurement sphere from the origin to a new fixed
        position.

        Soft-iron effects:
        Description: these distortions depend on the orientation of the sensor in relation to magnetic fields or
        external forces.
        Cause: they are caused by inhomogeneous ferromagnetic materials around the sensor. These materials distort
        the magnetic field according to their own magnetic permeability, creating anisotropic distortion.
        Consequence: instead of a perfect sphere, the sensor's raw measurements form an ellipsoid, where the axes
        are stretched or compressed depending on the direction of distortion.
    */
    transformation_mat: Matrix, // Transformation matrix which used to correct measurement distorsions (hard-iron and soft-iron effects, scale factors and axes misalignments).

    bias: Vector<f32>,  // Vector which contains static bias of each axis of the sensor that should be remove from raw measurements.
    raw_measurements: Vector<f32>,
    corrected_measurements: Vector<f32>,
    //conversion_factor: f32,
    initialized: bool,
}

impl Magnetometer {
    pub fn new() -> Result<Self, OpenAHRSError> {
        let mut mag = Self {
            transformation_mat: Matrix::new(),
            bias: Vector::new(),
            raw_measurements: Vector::new(),
            corrected_measurements: Vector::new(),
            initialized: false,
            //conversion_factor: 1.0,
        };

        mag.transformation_mat.init(3, 3)?;
        mag.bias.init(3)?;
        mag.raw_measurements.init(3)?;
        mag.corrected_measurements.init(3)?;

        //mag.transformation_mat.fill(0.0)?;
        //mag.bias.fill(0.0)?;
        //mag.raw_measurements.fill(0.0)?;
        //mag.corrected_measurements.fill(0.0)?;

        Ok(mag)
    }

    pub fn init(
        self: &mut Self,
        x_axis_scaling_factor: f32,
        y_axis_scaling_factor: f32,
        z_axis_scaling_factor: f32,
        xy_axes_cross_scaling_factor: f32,
        xz_axes_cross_scaling_factor: f32,
        yz_axes_cross_scaling_factor: f32,
        x_axis_bias: f32,
        y_axis_bias: f32,
        z_axis_bias: f32
        ) -> Result<(), OpenAHRSError> {
            if self.initialized {
                // The magnetometer has already been configured.
                Err(OpenAHRSError::MagAlreadyInit)
            } else {
                // Set the scaling factors.
                self.transformation_mat.set_element(0, 0, x_axis_scaling_factor)?;
                self.transformation_mat.set_element(1, 1, y_axis_scaling_factor)?;
                self.transformation_mat.set_element(2, 2, z_axis_scaling_factor)?;

                // Set the cross-axis scaling factors.
                self.transformation_mat.set_element(0, 1, xy_axes_cross_scaling_factor)?;
                self.transformation_mat.set_element(1, 0, xy_axes_cross_scaling_factor)?;
                self.transformation_mat.set_element(0, 2, xz_axes_cross_scaling_factor)?;
                self.transformation_mat.set_element(2, 0, xz_axes_cross_scaling_factor)?;
                self.transformation_mat.set_element(1, 2, yz_axes_cross_scaling_factor)?;
                self.transformation_mat.set_element(2, 1, yz_axes_cross_scaling_factor)?;

                // Set the biases.
                self.bias.set_element(0, x_axis_bias)?;
                self.bias.set_element(1, y_axis_bias)?;
                self.bias.set_element(2, z_axis_bias)?;

                self.initialized = true;    // Set the initialization flag to true.

                Ok(())  // Return no error.
            }
    }

    fn correct(self: &mut Self) -> Result<(), OpenAHRSError> {
        //let raw_measurements: Matrix = vector_to_matrix(&self.raw_measurements)?;
        let mut raw_measurements: Matrix = Matrix::new();
        raw_measurements.init(3, 1)?;
        set_col(&mut raw_measurements, &self.corrected_measurements, 0)?;
        let bias: Matrix = vector_to_matrix(&self.bias)?;
        let mut corrected_measurements: Matrix = Matrix::new();
        corrected_measurements.init(3, 1)?;

        let mut temp: Matrix = Matrix::new();
        temp.init(3, 1)?;
        //temp.fill(0.0)?;

        temp.sub(&raw_measurements, &bias)?;
        corrected_measurements.mul(&self.transformation_mat, &temp)?;
        get_col(&temp, &mut self.corrected_measurements, 0)?;

        Ok(())
    }

    pub fn update(self: &mut Self, ax: f32, ay: f32, az: f32) -> Result<(), OpenAHRSError> {
        // Check that the magnetometer is configured.
        if !self.initialized {
            // The magnetometer is not initialized.
            Err(OpenAHRSError::MagNotInit)
        } else {
            self.raw_measurements.set_element(0, ax)?;
            self.raw_measurements.set_element(1, ay)?;
            self.raw_measurements.set_element(2, az)?;

            self.correct()?;

            Ok(())
        }
    }

    pub fn get_x_magnetic_field(self: &Self) -> Result<f32, OpenAHRSError> {
        // Check that the magnetometer is configured.
        if !self.initialized {
            // The magnetometer is not initialized.
            Err(OpenAHRSError::MagNotInit)
        } else {
            let mx: f32 = self.corrected_measurements.get_element(0)?;

            Ok(mx)
        }
    }
}
