extern crate quaternion;
extern crate linalg;
extern crate utils;
extern crate libm;

use quaternion::quaternion::Quaternion;
use linalg::matrix::Matrix;
use linalg::vector::Vector;
use linalg::common::EPSILON;
use libm::{cosf, sinf};
use utils::utils::factorial;
use crate::gyrometer::{GyrometerConfig, Gyrometer};
use crate::common::{OpenAHRSError, NumericalIntegrationMethod as NIM, calculate_omega_matrix};

#[derive(Debug)]
pub struct AR {
    gyr: Gyrometer,         // Sensor of the fitler.

    attitude: Vector<f32>,  // Estimated attitude by the filter.

    ts: f32,                // Sampling period.
    method: NIM,            // Numerical integration method.
    order: u8,              // Order of the numerical integration method.

    initialized: bool,      // Initialization status.
}

impl AR {
    /// This method is used to create a new AR filter.
    pub fn new() -> Result<Self, OpenAHRSError> {
        let ar = Self {
            gyr: Gyrometer::new()?,     // Create the sensor (only a gyrometer, no drift correction).

            attitude: Vector::new(),    // Create the vector that will be used as a quaternion.

            // Filter settings.
            ts: 0.01,                   // Default sampling period.
            method: NIM::ClosedForm,    // Default numerical integration method.
            order: 1,                   // Default order of the numerical integration method.

            initialized: false,         // Default initialization status.
        };

        Ok(ar)  // Return the new filter with no error.
    }

    // This function is used to initialize the AR filter.
    pub fn init(self: &mut Self,
        qw: f32, qx: f32, qy: f32, qz: f32, // Initial attitude (quaternion coordinates).
        gyrometer_config: GyrometerConfig,  // Gyrometer configuration.
        ts: f32,                            // Sampling period.
        method: NIM,                        // Numerical integration method.
        //order: Option<u8>                   // Order of the numerical integration method.
        order: u8
        ) -> Result<(), OpenAHRSError> {
            // Check if the filter has already been initialized.
            if self.initialized {
                // The filter has already been initialized.
                return Err(OpenAHRSError::ARFilterAlreadyInit); // Return an error.
            }

            self.gyr.init(gyrometer_config)?;       // Initialize the gyrometer.
            self.attitude.init(4)?;                 // Initialize the vector that will be used as quaternion.
            self.attitude.fillq(qw, qx, qy, qz)?;   // Set initial attitude.
            self.ts = ts;                           // Set sampling rate.
            self.method = method;                   // Set the numerical integration method that will be used to estimate the attitude.
            self.order = order;                     // Set the order of the method.

            self.initialized = true;                // Set initialization status flag to true.

            Ok(())  // Return no error.
    }

    // This function is used to update the AR filter.
    pub fn update(self: &mut Self, gx: f32, gy: f32, gz: f32) -> Result<(), OpenAHRSError> {
        // Check that the filter is initialized.
        if !self.initialized {
            // The filter is not initialized.
            return Err(OpenAHRSError::ARFilterNotInit); // Return an error.
        }

        let mut temp = Matrix::new();   // Create a temporary matrix
        temp.init(4, 4)?;               // Initialize it.
        temp.fill_identity()?;          // Configuring it into an identity matrix.

        self.gyr.update(gx, gy, gz)?;   // Update the gyrometer with raw measurements to correct them.

        // Retrieve corrected measurements.
        let p = self.gyr.get_x_angular_rate()?;
        let q = self.gyr.get_y_angular_rate()?;
        let r = self.gyr.get_z_angular_rate()?;

        // Retrieve quaternion's components of previous estimated attitude.
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
            let theta = w_norm * self.ts / 2.0;
            let mut omega = calculate_omega_matrix(p, q, r)?;   // Calculate the transformation matrix Ω(ω).

            // Closed form method (not very suitable for numerical implementations).
            if self.method == NIM::ClosedForm || self.method == NIM::TaylorSeries {
                if self.method == NIM::ClosedForm {
                    temp.mul_by_scalar_in_place(cosf(theta))?;
                    omega.mul_by_scalar_in_place(sinf(theta) / w_norm)?;
                    temp.add_in_place(&omega)?;
                } else if self.method == NIM::TaylorSeries {
                    omega.mul_by_scalar_in_place(0.5 * self.ts)?;

                    for n in 1..=self.order {
                        // S = 0.5 * dt * Omega
                        let mut s = omega.duplicate()?;
                        // A' = S^n / !n
                        s.power_elements(n as f32)?;
                        let factor = factorial(n);
                        s.mul_by_scalar_in_place(1.0 / factor as f32)?;
                        // A = A + A'
                        temp.add_in_place(&s)?;
                    }
                }

                let attitude = temp.mul_new(&self.attitude.convert_to_matrix()?)?;

                // TODO: use more efficient way.
                qw = attitude.get_element(0, 0)?;
                qx = attitude.get_element(1, 0)?;
                qy = attitude.get_element(2, 0)?;
                qz = attitude.get_element(3, 0)?;
            } else if self.method == NIM::Euler {
                w.reinit(4)?;
                //let mut w: Vector<f32> = Vector::new();
                //w.init(4)?;
                w.fillq(0.0, p, q, r)?;
                w.mul_by_scalar_in_place(0.5 * self.ts)?;

                let mut delta: Vector<f32> = Vector::new();
                delta.init(4)?;
                delta.mul(&w, &self.attitude)?;

                qw += delta.get_qw()?;
                qx += delta.get_qx()?;
                qy += delta.get_qy()?;
                qz += delta.get_qz()?;
            } else {
                // The chosen method is not correct.
                return Err(OpenAHRSError::ARMethodError);   // Return an error.
            }

            self.attitude.fillq(qw, qx, qy, qz)?;
            self.attitude.normalize()?;
        } else {
            // The system is not moving (rotating) so the attitude doesn't change.
        }

        Ok(())  // Return no error.
    }

    pub fn get_attitude(self: &Self) -> Result<Vector<f32>, OpenAHRSError> {
        // Check if the filter has already been initialized.
        if !self.initialized {
            // The filter has already been initialized.
            return Err(OpenAHRSError::ARFilterNotInit); // Return an error.
        }

        // TODO: use more efficient way.
        Ok(self.attitude.duplicate()?)  // Return estimated attitude with no error.
    }

    #[cfg(feature = "std")]
    pub fn print_attitude(self: &Self) -> Result<(), OpenAHRSError> {
        // Check if the filter has already been initialized.
        if !self.initialized {
            // The filter has already been initialized.
            return Err(OpenAHRSError::ARFilterNotInit); // Return an error.
        }

        self.attitude.print()?;

        Ok(())  // Return no error.
    }
}
