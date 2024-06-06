//find . -name "*.rs" -print0 | while IFS= read -r -d '' file; do lines=$(wc -l < "$file"); echo "$file: $lines"; done
// total_lines=$(find . -name "*.rs" -print0 | xargs -0 cat | wc -l); echo "Total des lignes de code: $total_lines"

extern crate quaternion;
extern crate linalg;
extern crate utils;
extern crate libm;

use quaternion::quaternion::Quaternion;
use linalg::matrix::{Matrix, copy_from, mul};
use linalg::vector::Vector;
use linalg::common::EPSILON;
use linalg::linalg::vector_to_matrix;
//use libm::{cos, sin, exp};
use libm::{cos, sin};
use utils::utils::factorial;
use crate::gyroscope::Gyroscope;
use crate::common::{CLOSED_FORM, TAYLOR_SERIES, EULER, OpenAHRSError};

#[derive(Debug)]
pub struct AR {
    gyr: Gyroscope,
    orientation: Quaternion,
    ts: f64,
    method: u8,
    order: u8,
    initialized: bool,
}

impl AR {
    pub fn new() -> Result<Self, OpenAHRSError> {
        let ar = Self {
            gyr: Gyroscope::new()?,
            orientation: Quaternion::new()?,
            ts: 0.01,
            method: 0,
            order: 1,
            initialized: false,
        };

        Ok(ar)
    }

    pub fn init(
        self: &mut Self,
        /* Initial orientation. */
        qw: f64, qx: f64, qy: f64, qz: f64,
        /* Gyroscope settings. */
        x_axis_scaling_correction_factor: f64, y_axis_scaling_correction_factor: f64, z_axis_scaling_correction_factor: f64,
        xy_axes_non_orthogonality_correction_factor: f64, xz_axes_non_orthogonality_correction_factor: f64,
        yx_axes_non_orthogonality_correction_factor: f64, yz_axes_non_orthogonality_correction_factor: f64,
        zx_axes_non_orthogonality_correction_factor: f64, zy_axes_non_orthogonality_correction_factor: f64,
        x_axis_static_bias: f64, y_axis_static_bias: f64, z_axis_static_bias: f64,
        ts: f64,
        method: u8,
        order: u8
        ) -> Result<(), OpenAHRSError> {
            self.gyr.init(
                x_axis_scaling_correction_factor, y_axis_scaling_correction_factor, z_axis_scaling_correction_factor,
                xy_axes_non_orthogonality_correction_factor, xz_axes_non_orthogonality_correction_factor,
                yx_axes_non_orthogonality_correction_factor, yz_axes_non_orthogonality_correction_factor,
                zx_axes_non_orthogonality_correction_factor, zy_axes_non_orthogonality_correction_factor,
                x_axis_static_bias, y_axis_static_bias, z_axis_static_bias
            )?;
            self.orientation.fill(qw, qx, qy, qz)?;
            self.ts = ts;
            self.method = method;
            self.order = order;
            self.initialized = true;

            Ok(())  // Return no error.
    }

    fn calculate_omega_matrix(p: f64, q: f64, r: f64) -> Result<Matrix, OpenAHRSError> {
        // Create the matrix.
        let mut omega: Matrix = Matrix::new();
        omega.init(4, 4)?;

        // Set elements of the first column.
        omega.set_element(0, 0, 0.0)?;
        omega.set_element(1, 0, p)?;
        omega.set_element(2, 0, q)?;
        omega.set_element(3, 0, r)?;

        // Set elements of the second column.
        omega.set_element(0, 1, -p)?;
        omega.set_element(1, 1, 0.0)?;
        omega.set_element(2, 1, -r)?;
        omega.set_element(3, 1, q)?;

        // Set elements of the third column.
        omega.set_element(0, 2, -q)?;
        omega.set_element(1, 2, r)?;
        omega.set_element(2, 2, 0.0)?;
        omega.set_element(3, 2, -p)?;

        // Set elements of the fourth column.
        omega.set_element(0, 3, -r)?;
        omega.set_element(1, 3, -q)?;
        omega.set_element(2, 3, p)?;
        omega.set_element(3, 3, 0.0)?;

        Ok(omega)
    }

    pub fn update(self: &mut Self, gx: f64, gy: f64, gz: f64) -> Result<(), OpenAHRSError> {
        let mut omega: Matrix = Matrix::new();
        omega.init(4, 4)?;
        let mut temp: Matrix = Matrix::new();
        temp.init(4, 4)?;
        temp.fill_identity()?;
        let mut w: Vector<f64> = Vector::new();
        w.init(3)?;

        self.gyr.update(gx, gy, gz)?;
        let p: f64 = self.gyr.get_x_angular_rate()?;
        let q: f64 = self.gyr.get_y_angular_rate()?;
        let r: f64 = self.gyr.get_z_angular_rate()?;

        w.set_element(0, p)?;
        w.set_element(1, q)?;
        w.set_element(2, r)?;
        let w: f64 = w.calculate_norm()?;

        if w > EPSILON {
            let theta: f64 = w * self.ts / 2.0_f64;
            omega.copy_from(&Self::calculate_omega_matrix(p, q, r)?)?;

            if self.method == CLOSED_FORM {
                temp.mul_by_scalar(cos(theta))?;
                omega.mul_by_scalar(sin(theta) / w)?;
                temp.add(&omega, &copy_from(&temp)?)?;
            } else if self.method == TAYLOR_SERIES {
                let mut s: Matrix = Matrix::new();
                s.init(4, 4)?;

                for n in 0..self.order {
                    // S = 0.5 * dt * Omega
                    s.copy_from(&omega)?;
                    s.mul_by_scalar(0.5 * self.ts)?;
                    // A' = S^n / !n
                    s.power_exponent(n as f64)?;
                    let factor: u64 = factorial(n);
                    s.mul_by_scalar(1.0 / factor as f64)?;
                    // A = A + A'
                    temp.add(&copy_from(&temp)?, &s)?;
                }
            } else if self.method == EULER {
                // Some code here.
            } else {
                // The chosen method is not correct.
                return Err(OpenAHRSError::AEMethodError);   // Return an error.
            }

            let orientation: Matrix = mul(&temp, &vector_to_matrix(&self.orientation.get_vect()?)?)?;
            let qw: f64 = orientation.get_element(0, 0)?;
            let qx: f64 = orientation.get_element(1, 0)?;
            let qy: f64 = orientation.get_element(2, 0)?;
            let qz: f64 = orientation.get_element(3, 0)?;

            /*
            let mut w: Quaternion = Quaternion::new()?;
            w.fill(0.0_f64, p, q, r)?;
            w.mul_by_scalar(0.5_f64 * self.ts)?;

            let mut delta: Quaternion = Quaternion::new()?;
            delta.mul(&w, &self.orientation)?;

            let qw: f64 = self.orientation.get_qw()? + delta.get_qw()?;
            let qx: f64 = self.orientation.get_qx()? + delta.get_qx()?;
            let qy: f64 = self.orientation.get_qy()? + delta.get_qy()?;
            let qz: f64 = self.orientation.get_qz()? + delta.get_qz()?;
            */

            self.orientation.fill(qw, qx, qy, qz)?;
            self.orientation.normalize()?;
        } else {
            // The system is not moving (rotating) so the attitude doesn't change.
            //self.orientation.fill_identity()?;
        }

        Ok(())  // Return no error.
    }

    #[cfg(feature = "std")]
    pub fn print_attitude(self: &Self) -> Result<(), OpenAHRSError> {
        self.orientation.print()?;

        Ok(())  // Return no error.
    }
}
