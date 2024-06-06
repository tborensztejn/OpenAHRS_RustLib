extern crate linalg;
extern crate quaternion;
extern crate libm;

use linalg::common::{LinalgError};

pub const CLOSED_FORM:      u8 = 1;
pub const TAYLOR_SERIES:    u8 = 2;
pub const EULER:            u8 = 3;
pub const ADAMS_BASHFORTH:  u8 = 4;
pub const RUNGE_KUTTA:      u8 = 5;

#[derive(Debug)]
#[derive(PartialEq)]
pub enum OpenAHRSError {
    GyrNotInit,
    GyrAlreadyInit,
    AccNotInit,
    AccAlreadyInit,
    MagNotInit,
    MagAlreadyInit,
    AEMethodError,
    LinalgError(LinalgError),
}

impl From<LinalgError> for OpenAHRSError {
    fn from(error: LinalgError) -> Self {
        OpenAHRSError::LinalgError(error)
    }
}

#[cfg(feature = "std")]
use linalg::matrix::Matrix;
#[cfg(feature = "std")]
use linalg::common::is_valid_cols_number;
#[cfg(feature = "std")]
use quaternion::quaternion::Quaternion;
#[cfg(feature = "std")]
use linalg::vector::dot_product;

#[cfg(feature = "std")]
//pub fn generate_random_attitudes(number: u8, ts: f32) -> Result<Matrix, OpenAHRSError> {
pub fn generate_random_attitudes(number: u8) -> Result<Matrix, OpenAHRSError> {
    //extern crate quaternion;
    extern crate rand;
    extern crate libm;

    use rand::prelude::*;
    use linalg::common::PI;
    use libm::{sqrt, cos, sin, acos, fabs};

    is_valid_cols_number(number)?;

    let mut attitudes: Matrix = Matrix::new();
    attitudes.init(4, number)?;
    let mut previous_quat: Quaternion = Quaternion::new()?;
    previous_quat.fill_identity()?;

    for n in 0..number {
        let u1: f32 = rand::thread_rng().gen();  // Generates a random number between 0 and 1.
        let u2: f32 = rand::thread_rng().gen();  // Generates a random number between 0 and 1.
        let u3: f32 = rand::thread_rng().gen();  // Generates a random number between 0 and 1.

        let s1: f32 = sqrt(1.0 - u1);
        let s2: f32 = sqrt(u1);
        let t1: f32 = 2.0 * PI * u2;
        let t2: f32 = 2.0 * PI * u3;

        let mut qw: f32 = s2 * cos(t2);
        let mut qx: f32 = s1 * sin(t1);
        let mut qy: f32 = s1 * cos(t1);
        let mut qz: f32 = s2 * sin(t2);

        let mut actual_quat: Quaternion = Quaternion::new()?;
        actual_quat.fill(qw, qx, qy, qz)?;


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
        let scalar: f32 = dot_product(&actual_quat.get_vect()?, &previous_quat.get_vect()?)?;
        let theta: f32 = acos(scalar);  // Angle between the axis of the current quaternion and that of the previous quaternion.

        /*
            If the angle between the two axes of rotation of the quaternions is too great, an intermediate quaternion must be determined by spherical interpolation,
            ensuring that theangle between the preceding quaternion and the interpolated quaternion does not exceed the maximum angle.
        */

        let max_angle: f32 = PI / 32.0_f32;

        if fabs(theta) > max_angle {
            let scale: f32 = max_angle / fabs(theta);
            let mut interpolated_quat: Quaternion = Quaternion::new()?;
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
