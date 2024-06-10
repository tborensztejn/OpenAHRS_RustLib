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
use crate::gyroscope::{GyroscopeConfig, Gyroscope};
use crate::common::{
    CLOSED_FORM,
    TAYLOR_SERIES,
    EULER,
    OpenAHRSError,
    calculate_omega_matrix,
};

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
            gyr: Gyroscope::new()?,             // Filter sensor (only a gyroscope, no drift correction).

            orientation: Quaternion::new()?,    // Estimated attitude by the filter.

            // Filter settings.
            ts: 0.01_f32,                       // Sampling period.
            method: 0_u8,                       // Numerical integration method.
            order: 1_u8,                        // Order of the numerical integration method.

            initialized: false,                 // Initialization status.
        };

        Ok(ar)  // Return the structure with no error.
    }

    pub fn init(self: &mut Self,
        qw: f32, qx: f32, qy: f32, qz: f32, // Initial orientation (quaternion coordinates).
        gyroscope_config: GyroscopeConfig,  // Gyroscope configuration.
        ts: f32,                            // Sampling period.
        method: u8,                         // Numerical integration method.
        order: u8                           // Order of the numerical integration method.
        ) -> Result<(), OpenAHRSError> {
            self.gyr.init(gyroscope_config)?;
            self.orientation.fill(qw, qx, qy, qz)?;
            self.ts = ts;
            self.method = method;
            self.order = order;
            self.initialized = true;

            Ok(())  // Return no error.
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
            let mut omega = calculate_omega_matrix(p, q, r)?;   // Calculate the transformation matrix Ω(ω).

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
