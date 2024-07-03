extern crate linalg;

use linalg::matrix::Matrix;
use linalg::common::{LinalgError};

//pub(crate) const MAX_ORDER: usize = 5;      /// Maximum order of numerical integration methods.

#[derive(Debug)]
#[derive(PartialEq)]
/// All numerical integration methods supported by OpenAHRS.
pub enum NumericalIntegrationMethod {
    ClosedForm,     // Closed form solution (not very suitable for numerical implementations).
    TaylorSeries,   //
    Euler,
    AdamsBashforth,
    RungeKutta,
}

#[derive(Debug)]
#[derive(PartialEq)]
/// All OpenAHRS errors.
pub enum OpenAHRSError {
    // Sensors errors.
    GyrNotInit,                 /// This error occurs when an attempt is made to use an uninitialised gyrometer.
    GyrAlreadyInit,             /// This error occurs when an attempt is made to initialise a gyro that has already been initialised.
    InvalidCorrectionMatrix,
    InvalidStaticBiases,
    AccNotInit,
    AccAlreadyInit,
    MagNotInit,
    MagAlreadyInit,

    // AR filter errors.
    ARMethodError,
    ARFilterNotInit,

    // AQUA filter errors.
    AQUAFilterAlreadyInit,
    AQUAFilterNotInit,
    InvalidQuaternion,
    InvalidAQUAMode,
    InvalidAQUAInterpolationTreshold,

    InvalidGyroRawMeasurements,




    LinalgError(LinalgError),
}

impl From<LinalgError> for OpenAHRSError {
    fn from(error: LinalgError) -> Self {
        OpenAHRSError::LinalgError(error)
    }
}

#[cfg(feature = "std")]
use linalg::common::is_valid_cols_number;
#[cfg(feature = "std")]
use quaternion::quaternion::Quaternion;
#[cfg(feature = "std")]
use linalg::vector::Vector;

#[cfg(feature = "std")]
pub fn generate_random_attitudes(number: u8) -> Result<Matrix, LinalgError> {
    extern crate quaternion;
    extern crate rand;
    extern crate libm;

    use rand::prelude::*;
    use linalg::common::PI;
    use libm::{sqrtf, cosf, sinf, acosf, fabsf};

    is_valid_cols_number(number)?;

    let mut attitudes = Matrix::new();
    attitudes.init(4, number)?;
    let mut previous_quat: Vector<f32> = Vector::new();
    previous_quat.init(4)?;
    previous_quat.fill_identity()?;

    for n in 0..number {
        let u1: f32 = rand::thread_rng().gen();  // Generates a random number between 0 and 1.
        let u2: f32 = rand::thread_rng().gen();  // Generates a random number between 0 and 1.
        let u3: f32 = rand::thread_rng().gen();  // Generates a random number between 0 and 1.

        let s1 = sqrtf(1.0_f32 - u1);
        let s2 = sqrtf(u1);
        let t1 = 2.0_f32 * PI * u2;
        let t2 = 2.0_f32 * PI * u3;

        let mut qw = s2 * cosf(t2);
        let mut qx = s1 * sinf(t1);
        let mut qy = s1 * cosf(t1);
        let mut qz = s2 * sinf(t2);

        let mut actual_quat: Vector<f32> = Vector::new();
        actual_quat.init(4)?;
        actual_quat.fillq(qw, qx, qy, qz)?;


        /*
            In a context where the time step is fixed and variations between two attitudes can be significant, leading to high rotation vectors, it is crucial to adopt strategies
            to manage these variations and maintain the stability and accuracy of quaternion integration. If the variations between two successive quaternions are too great, intermediate
            attitude interpolation is required to reduce the rotation vectors between fixed time steps. A common method for quaternion interpolation is Spherical Linear Interpolation (SLERP).
            In a context where the time step is fixed and variations between two attitudes can be significant, leading to high rotation vectors, it is crucial to adopt strategies to manage
            these variations and maintain the stability and accuracy of quaternion integration.

            If the variations between two successive quaternions are too great, intermediate attitude interpolation is required to reduce the rotation vectors between fixed time steps.
            A common method for quaternion interpolation is Spherical Linear Interpolation (SLERP). Using SLERP, we insert intermediate quaternions between the two generated quaternions,
            thus reducing the angular variation at each fixed time step.
        */


        // Perform scalar product.
        let scalar = actual_quat.dot_product(&previous_quat)?;
        let theta = acosf(scalar);  // Angle between the axis of the current quaternion and that of the previous quaternion.

        /*
            If the angle between the two axes of rotation of the quaternions is too great, an intermediate quaternion must be determined by spherical interpolation,
            ensuring that theangle between the preceding quaternion and the interpolated quaternion does not exceed the maximum angle.
        */

        let max_angle = PI / 64.0_f32;

        if fabsf(theta) > max_angle {
            let scale = max_angle / fabsf(theta);

            let mut interpolated_quat: Vector<f32> = Vector::new();
            interpolated_quat.init(4)?;
            interpolated_quat.slerp(&previous_quat, &actual_quat, scale)?;

            qw = interpolated_quat.get_qw()?;
            qx = interpolated_quat.get_qx()?;
            qy = interpolated_quat.get_qy()?;
            qz = interpolated_quat.get_qz()?;

            previous_quat.copy_from(&interpolated_quat)?;
        } else {
            previous_quat.copy_from(&actual_quat)?;
        }

        attitudes.set_element(0, n, qw)?;
        attitudes.set_element(1, n, qx)?;
        attitudes.set_element(2, n, qy)?;
        attitudes.set_element(3, n, qz)?;
    }

    Ok(attitudes)
}

/// This function is used to calculate the transformation matrix Ω(ω) from components of the angular velocity pseudovector.
pub(crate) fn calculate_omega_matrix(p: f32, q: f32, r: f32) -> Result<Matrix, OpenAHRSError> {
    // Create the matrix.
    let mut omega = Matrix::new();
    omega.init(4, 4)?;

    // Set elements of the first column of the Ω(ω) matrix.
    omega.set_element(0, 0, 0.0)?;
    omega.set_element(1, 0, p)?;
    omega.set_element(2, 0, q)?;
    omega.set_element(3, 0, r)?;

    // Set elements of the second column of the Ω(ω) matrix.
    omega.set_element(0, 1, -p)?;
    omega.set_element(1, 1, 0.0)?;
    omega.set_element(2, 1, -r)?;
    omega.set_element(3, 1, q)?;

    // Set elements of the third column of the Ω(ω) matrix.
    omega.set_element(0, 2, -q)?;
    omega.set_element(1, 2, r)?;
    omega.set_element(2, 2, 0.0)?;
    omega.set_element(3, 2, -p)?;

    // Set elements of the fourth column of the Ω(ω) matrix.
    omega.set_element(0, 3, -r)?;
    omega.set_element(1, 3, -q)?;
    omega.set_element(2, 3, p)?;
    omega.set_element(3, 3, 0.0)?;

    Ok(omega)
}
