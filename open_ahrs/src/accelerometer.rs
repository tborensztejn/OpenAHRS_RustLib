extern crate linalg;

use linalg::matrix::Matrix;
use linalg::vector::Vector;
use linalg::linalg::{vector_to_matrix, get_col, set_col};
use crate::common::OpenAHRSError;

#[derive(Debug)]
pub struct Accelerometer {
    transformation_mat: Matrix,
    bias: Vector<f64>,
    raw_measurements: Vector<f64>,
    corrected_measurements: Vector<f64>,
    //conversion_factor: f64,
    initialized: bool,
}

impl Accelerometer {
    pub fn new() -> Result<Self, OpenAHRSError> {
        let mut acc = Self {
            transformation_mat: Matrix::new(),
            bias: Vector::new(),
            raw_measurements: Vector::new(),
            corrected_measurements: Vector::new(),
            initialized: false,
        };

        acc.transformation_mat.init(3, 3)?;
        acc.bias.init(3)?;
        acc.raw_measurements.init(3)?;
        acc.corrected_measurements.init(3)?;

        //acc.transformation_mat.fill(0.0)?;
        //acc.bias.fill(0.0)?;
        //acc.raw_measurements.fill(0.0)?;
        //acc.corrected_measurements.fill(0.0)?;

        Ok(acc)
    }

    pub fn init(
        self: &mut Self,
        x_axis_scaling_factor: f64,
        y_axis_scaling_factor: f64,
        z_axis_scaling_factor: f64,
        xy_axes_cross_scaling_factor: f64,
        xz_axes_cross_scaling_factor: f64,
        yz_axes_cross_scaling_factor: f64,
        x_axis_bias: f64,
        y_axis_bias: f64,
        z_axis_bias: f64
        ) -> Result<(), OpenAHRSError> {
            if self.initialized {
                // The accelerometer has already been configured.
                Err(OpenAHRSError::AccAlreadyInit)
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

    pub fn update(self: &mut Self, ax: f64, ay: f64, az: f64) -> Result<(), OpenAHRSError> {
        // Check that the accelerometer is configured.
        if !self.initialized {
            // The accelerometer is not initialized.
            Err(OpenAHRSError::AccNotInit)
        } else {
            self.raw_measurements.set_element(0, ax)?;
            self.raw_measurements.set_element(1, ay)?;
            self.raw_measurements.set_element(2, az)?;

            self.correct()?;

            Ok(())
        }
    }

    pub fn get_x_acceleration(self: &Self) -> Result<f64, OpenAHRSError> {
        // Check that the accelerometer is configured.
        if !self.initialized {
            // The accelerometer is not initialized.
            Err(OpenAHRSError::AccNotInit)
        } else {
            let ax: f64 = self.corrected_measurements.get_element(0)?;

            Ok(ax)
        }
    }
}
