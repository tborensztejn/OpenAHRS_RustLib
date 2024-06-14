extern crate linalg;
extern crate libm;

//use linalg::vector::{Vector, dot_product};
use linalg::vector::Vector;
use linalg::matrix::Matrix;
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

    // This function is used to fill a quaternion from a vector of size (4 x 1).
    pub fn fill_from_vector(self: &mut Self, vect: &Vector<f32>) -> Result<(), LinalgError> {
        self.vect.copy_from(&vect)?;

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
            self.vect.mul_by_scalar(1.0_f32 / (norm * norm))?;
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

    // This function is used to extract the vector from the quaternion object (can be used to convert a quaternion to a vector of size (4 x 1).
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

    // This function is used to perform the linear interpolation (LERP) of two quaternions.
    pub fn lerp(self: &mut Self, quat1: &Self, quat2: &Self, alpha: f32) -> Result<(), LinalgError> {
        //self.vect.lerp(&quat1.get_vect()?, &quat2.get_vect()?, alpha)?;
        self.vect.lerp(&quat1.vect, &quat2.vect, alpha)?;

        Ok(())  // Return no error.
    }

    /*
    // This function is used to set the qw value of a quaternion.
    pub fn set_qw(self: &Self, qw: f32) -> Result<(), LinalgError> {
        self.vect.set_element(0, qw)?;  // Set the qw value.

        Ok(())  // Return no error.
    }
    */

    // This function is used to obtain the qw value of a quaternion.
    pub fn get_qw(self: &Self) -> Result<f32, LinalgError> {
        Ok(self.vect.get_element(0)?)
    }

    /*
    // This function is used to set the qx value of a quaternion.
    pub fn set_qx(self: &Self, qx: f32) -> Result<(), LinalgError> {
        self.vect.set_element(0, qx)?;  // Set the qx value.

        Ok(())  // Return no error.
    }
    */

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

// This function is used to perform the spherical interpolation (SLERP) of two quaternions.
pub fn slerp(quat1: &Quaternion, quat2: &Quaternion, alpha: f32) -> Result<Quaternion, LinalgError> {
    let mut quat: Quaternion = Quaternion::new()?;
    //quat.vect.slerp(&quat1.get_vect()?, &quat2.get_vect()?, alpha)?;
    quat.vect.slerp(&quat1.vect, &quat2.vect, alpha)?;

    Ok(quat)    // Return the interpolated quaternion with no error.
}

// This function is used to perform the linear interpolation (LERP) of two quaternions.
pub fn lerp(quat1: &Quaternion, quat2: &Quaternion, alpha: f32) -> Result<Quaternion, LinalgError> {
    let mut quat: Quaternion = Quaternion::new()?;
    //quat.vect.lerp(&quat1.get_vect()?, &quat2.get_vect()?, alpha)?;
    quat.vect.lerp(&quat1.vect, &quat2.vect, alpha)?;

    Ok(quat)    // Return the interpolated quaternion with no error.
}

// This function is used to duplicate/copy a quaternion.
pub fn copy_from(quat: &Quaternion) -> Result<Quaternion, LinalgError> {
    let mut copied_quat = Quaternion::new()?;
    copied_quat.vect.copy_from(&quat.vect)?;

    Ok(copied_quat) // Return the copy of the original quaternion with no error.
}

// This function is used to convert a quaternion from a vector of size (4 x 1).
pub fn vector_to_quaternion(vect: &Vector<f32>) -> Result<Quaternion, LinalgError> {
    let mut quat = Quaternion::new()?;  // Create the quaternion.
    quat.fill_from_vector(&vect)?;      // Fill it using the vector components.

    Ok(quat)    // Return the quaternion with no error.
}

// This function is used to calculate the rotation matrix (Direct Cosine Matrix - DCM) from a quaternion.
pub fn convert_to_dcm(quat: &Quaternion) -> Result<Matrix, LinalgError> {
    let mut dcm = Matrix::new();    // Create the rotation matrix.
    dcm.init(3, 3)?;                // Initialize it.

    let mut element = 0.0_f32;       // Create a temporary variable to store an element of the matrix.

    let qw = quat.get_qw()?;        // Retrieve qw component from the quaternion.
    let qx = quat.get_qx()?;        // Retrieve qx component from the quaternion.
    let qy = quat.get_qy()?;        // Retrieve qy component from the quaternion.
    let qz = quat.get_qz()?;        // Retrieve qz component from the quaternion.

    // Fill the first column of the rotation matrix.
    //element = 1.0 - 2.0*(qy * qy + qz * qz);
    element = qw*qw + qx*qx - qy*qy - qz*qz;
    dcm.set_element(0, 0, element)?;
    element = 2.0*(qx*qy - qw*qz);
    dcm.set_element(0, 1, element)?;




    Ok(dcm) // Return the rotation matrix with no error.
}

pub trait Quat {
    fn is_quaternion(self: &Self) -> Result<bool, LinalgError>;
    fn set_qw(self: &mut Self, qw: f32) -> Result<(), LinalgError>;
    fn get_qw(self: &Self) -> Result<f32, LinalgError>;
    fn set_qx(self: &mut Self, qx: f32) -> Result<(), LinalgError>;
    fn get_qx(self: &Self) -> Result<f32, LinalgError>;
    fn set_qy(self: &mut Self, qy: f32) -> Result<(), LinalgError>;
    fn get_qy(self: &Self) -> Result<f32, LinalgError>;
    fn set_qz(self: &mut Self, qz: f32) -> Result<(), LinalgError>;
    fn get_qz(self: &Self) -> Result<f32, LinalgError>;
    fn fillq(self: &mut Self, qw: f32, qx: f32, qy: f32, qz: f32) -> Result<(), LinalgError>;
    fn conjugate(self: &mut Self) -> Result<(), LinalgError>;
    fn mul(self: &mut Self, quat1: &Self, quat2: &Self) -> Result<(), LinalgError>;
    fn fill_identity(self: &mut Self) -> Result<(), LinalgError>;
    fn invert(self: &mut Self) -> Result<(), LinalgError>;
}

impl Quat for Vector<f32> {
    // This function is used to check if a vector can be used as a quaternion.
    fn is_quaternion(self: &Self) -> Result<bool, LinalgError> {
        // Check that the vector size is 4x1.
        if self.get_rows()? != 4 {
            Ok(false)
        } else {
            Ok(true)
        }
    }

    // This function is used to set the qw value of a quaternion.
    fn set_qw(self: &mut Self, qw: f32) -> Result<(), LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::QuaternionSizeMismatch) // Return an error.
        }

        self.set_element(0, qw)?;   // Set the qw value.

        Ok(())  // Return no error.
    }

    // This function is used to retrieve the qw value of a quaternion.
    fn get_qw(self: &Self) -> Result<f32, LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::QuaternionSizeMismatch) // Return an error.
        }

        Ok(self.get_element(0)?)
    }

    // This function is used to set the qx value of a quaternion.
    fn set_qx(self: &mut Self, qx: f32) -> Result<(), LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::QuaternionSizeMismatch) // Return an error.
        }

        self.set_element(1, qx)?;   // Set the qx value.

        Ok(())  // Return no error.
    }

    // This function is used to retrieve the qx value of a quaternion.
    fn get_qx(self: &Self) -> Result<f32, LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::QuaternionSizeMismatch) // Return an error.
        }

        Ok(self.get_element(1)?)
    }

    // This function is used to set the qy value of a quaternion.
    fn set_qy(self: &mut Self, qy: f32) -> Result<(), LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::QuaternionSizeMismatch) // Return an error.
        }

        self.set_element(2, qy)?;   // Set the qy value.

        Ok(())  // Return no error.
    }

    // This function is used to retrieve the qy value of a quaternion.
    fn get_qy(self: &Self) -> Result<f32, LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::QuaternionSizeMismatch) // Return an error.
        }

        Ok(self.get_element(2)?)
    }

    // This function is used to set the qz value of a quaternion.
    fn set_qz(self: &mut Self, qz: f32) -> Result<(), LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::QuaternionSizeMismatch) // Return an error.
        }

        self.set_element(3, qz)?;   // Set the qz value.

        Ok(())  // Return no error.
    }

    // This function is used to retrieve the qy value of a quaternion.
    fn get_qz(self: &Self) -> Result<f32, LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::QuaternionSizeMismatch) // Return an error.
        }

        Ok(self.get_element(3)?)
    }

    // This function is used to fill a quaternion.
    fn fillq(self: &mut Self, qw: f32, qx: f32, qy: f32, qz: f32) -> Result<(), LinalgError> {
        /*
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::QuaternionSizeMismatch) // Return an error.
        }

        // Fill the quaternion.
        self.set_element(0, qw)?;
        self.set_element(1, qx)?;
        self.set_element(2, qy)?;
        self.set_element(3, qz)?;
        */

        // Fill the quaternion.
        self.set_qw(qw)?;
        self.set_qx(qx)?;
        self.set_qy(qy)?;
        self.set_qz(qz)?;

        Ok(())  // Return no error.
    }

    // This function is used to conjugate a quaternion.
    fn conjugate(self: &mut Self) -> Result<(), LinalgError> {
        /*
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::QuaternionSizeMismatch) // Return an error.
        }

        // Retrieve components of the quaternion.
        let qx = self.get_element(1)?;
        let qy = self.get_element(2)?;
        let qz = self.get_element(3)?;

        // Conjugate the quaternion.
        self.set_element(1, -qx)?;
        self.set_element(2, -qy)?;
        self.set_element(3, -qz)?;
        */

        let qx = self.get_qx()?;
        let qy = self.get_qy()?;
        let qz = self.get_qz()?;

        self.set_qx(-qx)?;
        self.set_qy(-qy)?;
        self.set_qz(-qz)?;

        Ok(())  // Return no error.
    }

    // This function is used to perform the multiplication operation (Hamilton product) of two quaternions.
    fn mul(self: &mut Self, quat1: &Self, quat2: &Self) -> Result<(), LinalgError> {
        /*
        // Check that these are all quaternions.
        if !self.is_quaternion()? || !quat1.is_quaternion()? || !quat2.is_quaternion()? {
            return Err(LinalgError::QuaternionSizeMismatch) // Return an error.
        }

        // Extract components of the first quaternion (quat1).
        let qw1 = quat1.get_element(0)?;
        let qx1 = quat1.get_element(1)?;
        let qy1 = quat1.get_element(2)?;
        let qz1 = quat1.get_element(3)?;

        // Extract components of the second quaternion (quat2).
        let qw2 = quat2.get_element(0)?;
        let qx2 = quat2.get_element(1)?;
        let qy2 = quat2.get_element(2)?;
        let qz2 = quat2.get_element(3)?;

        // Calculate the components of the resulting quaternion (self).
        let qw = qw1*qw2 - qx1*qx2 - qy1*qy2 - qz1*qz2;
        let qx = qw1*qx2 + qx1*qw2 - qz1*qy2 + qy1*qz2;
        let qy = qw1*qy2 + qz1*qx2 + qw2*qy1 - qx1*qz2;
        let qz = qw1*qz2 - qy1*qx2 + qx1*qy2 + qw2*qz1;

        // Store the calculated components in the resulting quaternion.
        self.set_element(0, qw)?;
        self.set_element(1, qx)?;
        self.set_element(2, qy)?;
        self.set_element(3, qz)?;
        */

        // Extract components of the first quaternion (quat1).
        let qw1 = quat1.get_qw()?;
        let qx1 = quat1.get_qx()?;
        let qy1 = quat1.get_qy()?;
        let qz1 = quat1.get_qz()?;

        // Extract components of the second quaternion (quat2).
        let qw2 = quat2.get_qw()?;
        let qx2 = quat2.get_qx()?;
        let qy2 = quat2.get_qy()?;
        let qz2 = quat2.get_qz()?;

        // Calculate the components of the resulting quaternion (self).
        let qw = qw1*qw2 - qx1*qx2 - qy1*qy2 - qz1*qz2;
        let qx = qw1*qx2 + qx1*qw2 - qz1*qy2 + qy1*qz2;
        let qy = qw1*qy2 + qz1*qx2 + qw2*qy1 - qx1*qz2;
        let qz = qw1*qz2 - qy1*qx2 + qx1*qy2 + qw2*qz1;

        self.fillq(qw, qx, qy, qz)?;

        Ok(())  // Return no error.
    }

    // This function is used to fill a quaternion as identity quaternion.
    fn fill_identity(self: &mut Self) -> Result<(), LinalgError> {
        self.fillq(1.0, 0.0, 0.0, 0.0)?;

        Ok(())  // Return no error.
    }

    // This function is used to invert a quaternion.
    fn invert(self: &mut Self) -> Result<(), LinalgError> {
        let norm = self.calculate_norm()?;  // Calculate the norm of the quaternion.
        self.conjugate()?;                  // Conjugate the quaternion.

        if norm > EPSILON {
            self.mul_by_scalar(1.0_f32 / (norm * norm))?;
        } else {
            self.fillq(0.0, 0.0, 0.0, 0.0)?;
        }

        Ok(())  // Return no error.
    }
}
