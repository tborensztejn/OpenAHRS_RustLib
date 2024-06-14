extern crate quaternion;
extern crate linalg;
extern crate utils;
extern crate libm;

//use quaternion::quaternion::Quaternion;
use quaternion::quaternion::Quat;
use linalg::matrix::{Matrix, copy_from, mul};
use linalg::vector::Vector;
use linalg::common::EPSILON;
use linalg::linalg::vector_to_matrix;
use libm::{cosf, sinf};
use utils::utils::factorial;
use crate::gyrometer::{GyrometerConfig, Gyrometer};
use crate::common::{
    CLOSED_FORM,
    TAYLOR_SERIES,
    EULER,
    OpenAHRSError,
    calculate_omega_matrix,
};

#[derive(Debug)]
pub struct AR {
    gyr: Gyrometer,

    //attitude: Quaternion,
    attitude: Vector<f32>,

    ts: f32,
    method: u8,
    order: u8,

    initialized: bool,
}

impl AR {
    pub fn new() -> Result<Self, OpenAHRSError> {
        let ar = Self {
            gyr: Gyrometer::new()?,         // Filter sensor (only a gyrometer, no drift correction).

            //attitude: Quaternion::new()?,   // Estimated attitude by the filter.
            attitude: Vector::new(),         // Estimated attitude by the filter.

            // Filter settings.
            ts: 0.01_f32,                   // Sampling period.
            method: 0_u8,                   // Numerical integration method.
            order: 1_u8,                    // Order of the numerical integration method.

            initialized: false,             // Initialization status.
        };

        Ok(ar)  // Return the structure with no error.
    }

    // This function is used to initialize the AR filter.
    pub fn init(self: &mut Self,
        qw: f32, qx: f32, qy: f32, qz: f32, // Initial attitude (quaternion coordinates).
        gyrometer_config: GyrometerConfig,  // Gyrometer configuration.
        ts: f32,                            // Sampling period.
        method: u8,                         // Numerical integration method.
        order: u8                           // Order of the numerical integration method.
        ) -> Result<(), OpenAHRSError> {
            self.gyr.init(gyrometer_config)?;       // Initialize the gyrometer.
            self.attitude.init(4)?;                 // Initialize the vector that will be used as quaternion.
            //self.attitude.fill(qw, qx, qy, qz)?;    // Set initial attitude.
            self.attitude.fillq(qw, qx, qy, qz)?;   // Set initial attitude.
            self.ts = ts;                           // Set sampling rate.
            self.method = method;                   // Set the numerical integration method that will be used to estimate the attitude.
            self.order = order;                     // Set the order of the method.

            self.initialized = true;    // Set initialization status flag to true.

            Ok(())  // Return no error.
    }

    // This function is used to update the AR filter.
    pub fn update(self: &mut Self, gx: f32, gy: f32, gz: f32) -> Result<(), OpenAHRSError> {
        let mut temp = Matrix::new();   // Create a temporary matrix
        temp.init(4, 4)?;               // Initialize it.
        temp.fill_identity()?;          // Configuring it into an identity matrix.

        self.gyr.update(gx, gy, gz)?;   // Update the gyrometer with raw measurements to correct them.

        // Retrieve corrected measurements.
        let p = self.gyr.get_x_angular_rate()?;
        let q = self.gyr.get_y_angular_rate()?;
        let r = self.gyr.get_z_angular_rate()?;

        let mut qw = self.attitude.get_qw()?;
        let mut qx = self.attitude.get_qx()?;
        let mut qy = self.attitude.get_qy()?;
        let mut qz = self.attitude.get_qz()?;

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

                //let attitude = mul(&temp, &vector_to_matrix(&self.attitude.get_vect()?)?)?;
                let attitude = mul(&temp, &vector_to_matrix(&self.attitude)?)?;

                // TODO: use more efficient way.
                qw = attitude.get_element(0, 0)?;
                qx = attitude.get_element(1, 0)?;
                qy = attitude.get_element(2, 0)?;
                qz = attitude.get_element(3, 0)?;
            } else if self.method == EULER {
                //let mut w = Quaternion::new()?;
                w.reinit(4)?;
                //let mut w: Vector<f32> = Vector::new();
                //w.init(4)?;
                //w.fill(0.0_f32, p, q, r)?;
                w.fillq(0.0_f32, p, q, r)?;
                w.mul_by_scalar(0.5_f32 * self.ts)?;

                //let mut delta = Quaternion::new()?;
                let mut delta: Vector<f32> = Vector::new();
                delta.init(4)?;
                delta.mul(&w, &self.attitude)?;

                qw += delta.get_qw()?;
                qx += delta.get_qx()?;
                qy += delta.get_qy()?;
                qz += delta.get_qz()?;
            } else {
                // The chosen method is not correct.
                return Err(OpenAHRSError::AEMethodError);   // Return an error.
            }

            //self.attitude.fill(qw, qx, qy, qz)?;
            self.attitude.fillq(qw, qx, qy, qz)?;
            self.attitude.normalize()?;
        } else {
            // The system is not moving (rotating) so the attitude doesn't change.
        }

        Ok(())  // Return no error.
    }

    #[cfg(feature = "std")]
    pub fn print_attitude(self: &Self) -> Result<(), OpenAHRSError> {
        self.attitude.print()?;

        Ok(())  // Return no error.
    }
}
