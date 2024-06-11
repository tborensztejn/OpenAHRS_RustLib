extern crate linalg;
extern crate libm;

//use linalg::vector::{Vector, dot_product};
use linalg::vector::Vector;
use linalg::common::{EPSILON, LinalgError};
//use libm::{acosf, sinf};

#[derive(Debug)]
pub struct Quaternion {
    pub(crate) vect: Vector<f32>,
}

impl Quaternion {
    // This function is used to create a new quaternion.
    pub fn new() -> Result<Self, LinalgError> {
        let mut quat = Self {
            vect: Vector::new(),
        };

        quat.vect.init(4)?;

        Ok(quat)    // Return the new quaternion.
    }

    // This function is used to fill a quaternion.
    pub fn fill(self: &mut Self, qw: f32, qx: f32, qy: f32, qz: f32) -> Result<(), LinalgError> {
        self.vect.set_element(0, qw)?;
        self.vect.set_element(1, qx)?;
        self.vect.set_element(2, qy)?;
        self.vect.set_element(3, qz)?;

        Ok(())  // Return no error.
    }

    // This function is used to duplicate/copy a quaternion.
    pub fn copy_from(self: &mut Self, other: &Self) -> Result<(), LinalgError> {
        self.vect.copy_from(&other.vect)?;

        Ok(())  // Return no error.
    }

    // This function is used to conjugate a quaternion.
    pub fn conjugate(self: &mut Self) -> Result<(), LinalgError> {
        let qx = self.vect.get_element(1)?;
        let qy = self.vect.get_element(2)?;
        let qz = self.vect.get_element(3)?;

        self.vect.set_element(1, -qx)?;
        self.vect.set_element(2, -qy)?;
        self.vect.set_element(3, -qz)?;

        Ok(())  // Return no error.
    }

    // This function is used to perform the addition operation of two quaternions.
    pub fn add(self: &mut Self, quat1: &Self, quat2: &Self) -> Result<(), LinalgError> {
        self.vect.add(&quat1.vect, &quat2.vect)?;

        Ok(())  // Return no error.
    }

    // This function is used to add another quaternion to itself.
    pub fn add_in_place(self: &mut Self, other: &Self) -> Result<(), LinalgError> {
        self.vect.add_in_place(&other.vect)?;

        Ok(())  // Return no error.
    }

    // This function is used to perform the subtraction operation of two quaternions.
    pub fn sub(self: &mut Self, quat1: &Self, quat2: &Self) -> Result<(), LinalgError> {
        self.vect.sub(&quat1.vect, &quat2.vect)?;

        Ok(())  // Return no error.
    }

    // This function is used to subtract another quaternion to itself.
    pub fn sub_in_place(self: &mut Self, other: &Self) -> Result<(), LinalgError> {
        self.vect.sub_in_place(&other.vect)?;

        Ok(())  // Return no error.
    }

    // This function is used to perform the multiplication operation (Hamilton product) of two quaternions.
    pub fn mul(self: &mut Self, quat1: &Self, quat2: &Self) -> Result<(), LinalgError> {
        // Extract components of the first quaternion (quat1).
        let qw1 = quat1.vect.get_element(0)?;
        let qx1 = quat1.vect.get_element(1)?;
        let qy1 = quat1.vect.get_element(2)?;
        let qz1 = quat1.vect.get_element(3)?;

        // Extract components of the second quaternion (quat2).
        let qw2 = quat2.vect.get_element(0)?;
        let qx2 = quat2.vect.get_element(1)?;
        let qy2 = quat2.vect.get_element(2)?;
        let qz2 = quat2.vect.get_element(3)?;

        // Calculate the components of the resulting quaternion (self).
        let qw = qw1 * qw2 - qx1 * qx2 - qy1 * qy2 - qz1 * qz2;
        let qx = qw1 * qx2 + qx1 * qw2 - qz1 * qy2 + qy1 * qz2;
        let qy = qw1 * qy2 + qz1 * qx2 + qw2 * qy1 - qx1 * qz2;
        let qz = qw1 * qz2 - qy1 * qx2 + qx1 * qy2 + qw2 * qz1;

        // Store the components of in the resulting quaternion.
        self.fill(qw, qx, qy, qz)?;

        Ok(())  // Return no error.
    }

    // This function is used to normalize a quaternion.
    pub fn normalize(self: &mut Self) -> Result<(), LinalgError> {
        self.vect.normalize()?;

        Ok(())  // Return no error.
    }

    // This function is used to fill a quaternion as identity quaternion.
    pub fn fill_identity(self: &mut Self) -> Result<(), LinalgError> {
        self.fill(1.0, 0.0, 0.0, 0.0)?;

        Ok(())  // Return no error.
    }

    // This function is used to invert a quaternion.
    pub fn invert(self: &mut Self) -> Result<(), LinalgError> {
        let norm = self.vect.calculate_norm()?;    // Calculate the norm of the quaternion.
        self.conjugate()?;  // Conjugate the quaternion.

        if norm > EPSILON {
            self.vect.mul_by_scalar(1.0 / (norm * norm))?;
        } else {
            self.fill(0.0, 0.0, 0.0, 0.0)?;
        }

        Ok(())  // Return no error.
    }

    // This function is used to multiply by a scalar all elements of a quaternion.
    pub fn mul_by_scalar(self: &mut Self, scalar: f32) -> Result<(), LinalgError> {
        self.vect.mul_by_scalar(scalar)?;

        Ok(())  // Return no error.
    }

    // This function is used to extract the vector from the quaternion object (can be used to convert a quaternion to a vector of size (4 x 1).).
    pub fn get_vect(self: &Self) -> Result<Vector<f32>, LinalgError> {
        let mut vect: Vector<f32> = Vector::new();
        vect.init(4)?;
        vect.copy_from(&self.vect)?;

        Ok(vect)
    }

    /*
    // This function is used to perform the spherical interpolation (SLERP) of two quaternions.
    pub fn slerp(self: &mut Self, quat1: &Self, quat2: &Self, alpha: f32) -> Result<(), LinalgError> {
        // Perform scalar product.
        let scalar = dot_product(&quat1.get_vect()?, &quat2.get_vect()?)?;
        let theta = acosf(scalar);  // Angle between the axis of the first quaternion and the second one.

        // Create a copy of the first quaternion.
        let mut q1 = Self::new()?;
        q1.copy_from(&quat1)?;

        // Create a copy of the second quaternion.
        let mut q2 = Self::new()?;
        q2.copy_from(&quat2)?;

        q1.mul_by_scalar(sinf((1.0_f32 - alpha)*theta) / sinf(theta))?;
        q2.mul_by_scalar(sinf(alpha*theta) / sinf(theta))?;

        self.add(&q1, &q2)?;

        Ok(())    // Return no error.
    }
    */

    // This function is used to perform the spherical interpolation (SLERP) of two quaternions.
    pub fn slerp(self: &mut Self, quat1: &Self, quat2: &Self, alpha: f32) -> Result<(), LinalgError> {
        //self.vect.slerp(&quat1.get_vect()?, &quat2.get_vect()?, alpha)?;
        self.vect.slerp(&quat1.vect, &quat2.vect, alpha)?;

        Ok(())  // Return no error.
    }

    // This function is used to obtain the qw value of a quaternion.
    pub fn get_qw(self: &Self) -> Result<f32, LinalgError> {
        Ok(self.vect.get_element(0)?)
    }

    // This function is used to obtain the qx value of a quaternion.
    pub fn get_qx(self: &Self) -> Result<f32, LinalgError> {
        Ok(self.vect.get_element(1)?)
    }

    // This function is used to obtain the qy value of a quaternion.
    pub fn get_qy(self: &Self) -> Result<f32, LinalgError> {
        Ok(self.vect.get_element(2)?)
    }

    // This function is used to obtain the qz value of a quaternion.
    pub fn get_qz(self: &Self) -> Result<f32, LinalgError> {
        Ok(self.vect.get_element(3)?)
    }

    #[cfg(feature = "std")]
    pub fn print(self: &Self) -> Result<(), LinalgError> {
        self.vect.print()?;

        Ok(())  // Return no error.
    }
}

/*
// This function is used to perform the spherical interpolation (SLERP) of two quaternions.
pub fn slerp(quat1: &Quaternion, quat2: &Quaternion, alpha: f32) -> Result<Quaternion, LinalgError> {
    // Perform scalar product.
    let scalar = dot_product(&quat1.get_vect()?, &quat2.get_vect()?)?;
    let theta = acosf(scalar);  // Angle between the axis of the first quaternion and the second one.

    // Create a copy of the first quaternion.
    let mut q1: Quaternion = Quaternion::new()?;
    q1.copy_from(&quat1)?;

    // Create a copy of the second quaternion.
    let mut q2: Quaternion = Quaternion::new()?;
    q2.copy_from(&quat2)?;

    q1.mul_by_scalar(sinf((1.0_f32 - alpha)*theta) / sinf(theta))?;
    q2.mul_by_scalar(sinf(alpha*theta) / sinf(theta))?;

    let mut q: Quaternion = Quaternion::new()?;
    q.add(&q1, &q2)?;

    Ok(q)   // Return the interpolated quaternion with no error.
}
*/

pub fn slerp(quat1: &Quaternion, quat2: &Quaternion, alpha: f32) -> Result<Quaternion, LinalgError> {
    let mut quat: Quaternion = Quaternion::new()?;
    //quat.vect.slerp(&quat1.get_vect()?, &quat2.get_vect()?, alpha)?;
    quat.vect.slerp(&quat1.vect, &quat2.vect, alpha)?;

    Ok(quat)    // Return the interpolated quaternion with no error.
}
