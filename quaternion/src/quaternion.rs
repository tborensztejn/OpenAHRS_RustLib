extern crate linalg;
extern crate utils;

use linalg::vector::Vector;
use linalg::matrix::Matrix;
use linalg::common::{EPSILON, LinalgError};
use utils::utils::close_all;

pub trait Quaternion {
    fn is_quaternion(self: &Self) -> Result<bool, LinalgError>;
    fn is_real(self: &Self) -> Result<bool, LinalgError>;
    fn is_pure(self: &Self) -> Result<bool, LinalgError>;
    fn set_qw(self: &mut Self, qw: f32) -> Result<(), LinalgError>;
    fn get_qw(self: &Self) -> Result<f32, LinalgError>;
    fn set_qx(self: &mut Self, qx: f32) -> Result<(), LinalgError>;
    fn get_qx(self: &Self) -> Result<f32, LinalgError>;
    fn set_qy(self: &mut Self, qy: f32) -> Result<(), LinalgError>;
    fn get_qy(self: &Self) -> Result<f32, LinalgError>;
    fn set_qz(self: &mut Self, qz: f32) -> Result<(), LinalgError>;
    fn get_qz(self: &Self) -> Result<f32, LinalgError>;
    fn get_vector_part(self: &Self) -> Result<Vector<f32>, LinalgError>;
    fn fillq(self: &mut Self, qw: f32, qx: f32, qy: f32, qz: f32) -> Result<(), LinalgError>;
    fn conjugate(self: &mut Self) -> Result<(), LinalgError>;
    fn mul(self: &mut Self, quat1: &Self, quat2: &Self) -> Result<(), LinalgError>;
    fn fill_identity(self: &mut Self) -> Result<(), LinalgError>;
    fn invert(self: &mut Self) -> Result<(), LinalgError>;
    fn convert_to_dcm(self: &mut Self) -> Result<Matrix, LinalgError>;
    //fn exp(self: &mut Self) -> Result<(), LinalgError>;
    //fn log(self: &mut Self) -> Result<(), LinalgError>;
    //fn is_versor(self: &Self) -> Result<bool, LinalgError>;
    //fn is_identity(self: &Self) -> Result<bool, LinalgError>;
    //fn mult_L(self: &Self) -> Result<Matrix, LinalgError>;
    //fn mult_R(self: &Self) -> Result<Matrix, LinalgError>;
    //fn rotate(self: &Self, Self) -> Result<Self, LinalgError>;

}

impl Quaternion for Vector<f32> {
    // This function is used to check if a vector can be used as a quaternion.
    fn is_quaternion(self: &Self) -> Result<bool, LinalgError> {
        // Check that the vector size is 4x1.
        if self.get_rows()? != 4 {
            Ok(false)
        } else {
            Ok(true)
        }
    }

    fn is_real(self: &Self) -> Result<bool, LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::QuaternionSizeMismatch) // Return an error.
        }

        // Check if the components of the scalar part are equal to 0.
        for row in 1..4 {
            if !close_all(self.get_element(row)?, 0.0_f32, EPSILON).map_err(LinalgError::UtilsError)? {
                return Ok(false);   // Return false wwith no error.
            }
        }

        Ok(true)    // Return true with no error.
    }

    fn is_pure(self: &Self) -> Result<bool, LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::QuaternionSizeMismatch) // Return an error.
        }

        // Check if the real part componentof the quaternion is equal to 0.
        if !close_all(self.get_element(0)?, 0.0_f32, EPSILON).map_err(LinalgError::UtilsError)? {
            return Ok(false);   // Return false wwith no error.
        }

        Ok(true)    // Return true with no error.
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

    // This function is used to retrieve the qz value of a quaternion.
    fn get_qz(self: &Self) -> Result<f32, LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::QuaternionSizeMismatch) // Return an error.
        }

        Ok(self.get_element(3)?)
    }

    // This function is used to retrieve the vector part of a quaternion.
    fn get_vector_part(self: &Self) -> Result<Vector<f32>, LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::QuaternionSizeMismatch) // Return an error.
        }

        let mut qv: Vector<f32> = Vector::new();
        qv.init(3)?;

        let x = self.get_element(1)?;
        let y = self.get_element(2)?;
        let z = self.get_element(3)?;

        qv.set_element(0, x)?;
        qv.set_element(1, y)?;
        qv.set_element(2, z)?;

        Ok(qv)
    }

    // This function is used to fill a quaternion.
    fn fillq(self: &mut Self, qw: f32, qx: f32, qy: f32, qz: f32) -> Result<(), LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::QuaternionSizeMismatch) // Return an error.
        }

        // Fill the quaternion.
        self.set_element(0, qw)?;
        self.set_element(1, qx)?;
        self.set_element(2, qy)?;
        self.set_element(3, qz)?;

        Ok(())  // Return no error.
    }

    // This function is used to conjugate a quaternion.
    fn conjugate(self: &mut Self) -> Result<(), LinalgError> {
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

        Ok(())  // Return no error.
    }

    // This function is used to perform the multiplication operation (Hamilton product) of two quaternions.
    fn mul(self: &mut Self, quat1: &Self, quat2: &Self) -> Result<(), LinalgError> {
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

    // This function is used to calculate the rotation matrix (Direct Cosine Matrix - DCM) from a quaternion.
    fn convert_to_dcm(self: &mut Self) -> Result<Matrix, LinalgError> {
        let mut dcm = Matrix::new();    // Create the rotation matrix.
        dcm.init(3, 3)?;                // Initialize it.

        let mut element = 0.0_f32;      // Create a temporary variable to store an element of the matrix.

        let qw = self.get_qw()?;        // Retrieve qw component from the quaternion.
        let qx = self.get_qx()?;        // Retrieve qx component from the quaternion.
        let qy = self.get_qy()?;        // Retrieve qy component from the quaternion.
        let qz = self.get_qz()?;        // Retrieve qz component from the quaternion.

        // Fill the first column of the rotation matrix.
        //element = 1.0 - 2.0*(qy * qy + qz * qz);
        element = qw*qw + qx*qx - qy*qy - qz*qz;
        dcm.set_element(0, 0, element)?;
        element = 2.0*(qx*qy - qw*qz);
        dcm.set_element(0, 1, element)?;

        // Add some code here.

        Ok(dcm) // Return the rotation matrix with no error.
    }

    /*
    fn exp(self: &mut Self) -> Result<(), LinalgError> {
        // Check if the quaternion is real.
        if self.is_real()? {
            self.fillq(1.0, 0.0, 0.0, 0.0)?;
        }

        let qv = self.get_vector_part()?;
        qv_norm = calculate_norm()?;
        qv.normalize()?;



        // Check if the quaternion is pure.
        if self.is_pure()? {
            //
        }
        */
}
