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
use libm::{cosf, sinf};
use utils::utils::factorial;
use crate::gyroscope::Gyroscope;
use crate::common::{CLOSED_FORM, TAYLOR_SERIES, EULER, OpenAHRSError};

#[derive(Debug)]
pub struct AR {
    gyr: Gyroscope,
    orientation: Quaternion,
    ts: f32,
    method: u8,
    order: u8,
    initialized: bool,
}

impl AR {
    pub fn new() -> Result<Self, OpenAHRSError> {
        let ar = Self {
            gyr: Gyroscope::new()?,
            orientation: Quaternion::new()?,
            ts: 0.01_f32,
            method: 0_u8,
            order: 1_u8,
            initialized: false,
        };

        Ok(ar)
    }

    pub fn init(self: &mut Self,
        /* Initial orientation. */
        qw: f32, qx: f32, qy: f32, qz: f32,
        /* Gyroscope settings. */
        x_axis_scaling_correction_factor: f32, y_axis_scaling_correction_factor: f32, z_axis_scaling_correction_factor: f32,
        xy_axes_non_orthogonality_correction_factor: f32, xz_axes_non_orthogonality_correction_factor: f32,
        yx_axes_non_orthogonality_correction_factor: f32, yz_axes_non_orthogonality_correction_factor: f32,
        zx_axes_non_orthogonality_correction_factor: f32, zy_axes_non_orthogonality_correction_factor: f32,
        x_axis_static_bias: f32, y_axis_static_bias: f32, z_axis_static_bias: f32,
        ts: f32,
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

    fn calculate_omega_matrix(p: f32, q: f32, r: f32) -> Result<Matrix, OpenAHRSError> {
        // Create the matrix.
        let mut omega = Matrix::new();
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

    pub fn update(self: &mut Self, gx: f32, gy: f32, gz: f32) -> Result<(), OpenAHRSError> {
        let mut temp = Matrix::new();   // Create a temporary matrix
        temp.init(4, 4)?;               // Initialize it.
        temp.fill_identity()?;          // Configuring it into an identity matrix.

        self.gyr.update(gx, gy, gz)?;   // Update the gyroscope with raw measurements to correct them.

        // Retrieve corrected measurements.
        let p = self.gyr.get_x_angular_rate()?;
        let q = self.gyr.get_y_angular_rate()?;
        let r = self.gyr.get_z_angular_rate()?;

        let mut qw = self.orientation.get_qw()?;
        let mut qx = self.orientation.get_qx()?;
        let mut qy = self.orientation.get_qy()?;
        let mut qz = self.orientation.get_qz()?;

        let mut w: Vector<f32> = Vector::new(); // Create the angular velocity pseudovector.
        w.init(3)?;                             // Initialize it.

        // Fill it with gyro-corrected measurements.
        w.set_element(0, p)?;
        w.set_element(1, q)?;
        w.set_element(2, r)?;

        let w_norm = w.calculate_norm()?;   // Calculate its norm.

        // Check that the norm is not zero.
        if w_norm > EPSILON {
            let theta = w_norm * self.ts / 2.0_f32;
            let mut omega = Self::calculate_omega_matrix(p, q, r)?; // Calculate the transformation matrix Ω(ω).

            // Closed form method (not very suitable for numerical implementations).
            if self.method == CLOSED_FORM || self.method == TAYLOR_SERIES {
                if self.method == CLOSED_FORM {
                    temp.mul_by_scalar(cosf(theta))?;
                    omega.mul_by_scalar(sinf(theta) / w_norm)?;
                    temp.add_in_place(&omega)?;
                } else if self.method == TAYLOR_SERIES {
                    omega.mul_by_scalar(0.5_f32 * self.ts)?;

                    for n in 1..=self.order {
                        // S = 0.5 * dt * Omega
                        let mut s = copy_from(&omega)?;
                        // A' = S^n / !n
                        s.power_exponent(n as f32)?;
                        let factor = factorial(n);
                        s.mul_by_scalar(1.0_f32 / factor as f32)?;
                        // A = A + A'
                        temp.add_in_place(&s)?;
                    }
                }

                let orientation = mul(&temp, &vector_to_matrix(&self.orientation.get_vect()?)?)?;

                qw = orientation.get_element(0, 0)?;
                qx = orientation.get_element(1, 0)?;
                qy = orientation.get_element(2, 0)?;
                qz = orientation.get_element(3, 0)?;
            } else if self.method == EULER {
                let mut w = Quaternion::new()?;
                w.fill(0.0_f32, p, q, r)?;
                w.mul_by_scalar(0.5_f32 * self.ts)?;

                let mut delta = Quaternion::new()?;
                delta.mul(&w, &self.orientation)?;

                qw += delta.get_qw()?;
                qx += delta.get_qx()?;
                qy += delta.get_qy()?;
                qz += delta.get_qz()?;
            } else {
                // The chosen method is not correct.
                return Err(OpenAHRSError::AEMethodError);   // Return an error.
            }

            self.orientation.fill(qw, qx, qy, qz)?;
            self.orientation.normalize()?;
        } else {
            // The system is not moving (rotating) so the attitude doesn't change.
        }

        Ok(())  // Return no error.
    }

    #[cfg(feature = "std")]
    pub fn print_attitude(self: &Self) -> Result<(), OpenAHRSError> {
        self.orientation.print()?;

        Ok(())  // Return no error.
    }
}
