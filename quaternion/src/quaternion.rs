extern crate linalg;
extern crate utils;
extern crate libm;

use linalg::vector::Vector;
use linalg::matrix::Matrix;
use linalg::common::{EPSILON, LinalgError, E};
use utils::utils::{allclose, clip, Sign};
use libm::{sqrtf, atan2f, asinf, powf, cosf, sinf};

pub trait Quaternion {
    /// This method is used to check if a vector can be used as a quaternion.
    fn is_quaternion(self: &Self) -> Result<bool, LinalgError>;
    /// This method is used to check if a quaternion is real or not.
    fn is_real(self: &Self) -> Result<bool, LinalgError>;
    /// This method is used to check if a quaternion is pure or not.
    fn is_pure(self: &Self) -> Result<bool, LinalgError>;
    /// This method is used to check if a quaternion is a versor or not within a specified tolerance.
    fn is_versor(self: &Self) -> Result<bool, LinalgError>;
    /// This method is used to verify if a quaternion is the identity quaternion.
    fn is_identity(self: &Self) -> Result<bool, LinalgError>;
    /// This method is used to set w component of a quaternion.
    fn set_qw(self: &mut Self, qw: f32) -> Result<(), LinalgError>;
    /// This method is used to retrieve w component of a quaternion.
    fn get_qw(self: &Self) -> Result<f32, LinalgError>;
    /// This method is used to set x component of a quaternion.
    fn set_qx(self: &mut Self, qx: f32) -> Result<(), LinalgError>;
    /// This method is used to retrieve x component of a quaternion.
    fn get_qx(self: &Self) -> Result<f32, LinalgError>;
    /// This method is used to set y component of a quaternion.
    fn set_qy(self: &mut Self, qy: f32) -> Result<(), LinalgError>;
    /// This method is used to retrieve y component of a quaternion.
    fn get_qy(self: &Self) -> Result<f32, LinalgError>;
    /// This method is used to set z component of a quaternion.
    fn set_qz(self: &mut Self, qz: f32) -> Result<(), LinalgError>;
    /// This method is used to retrieve z component of a quaternion.
    fn get_qz(self: &Self) -> Result<f32, LinalgError>;
    /// This method is used to retrieve vector part of a quaternion.
    fn get_vector_part(self: &Self) -> Result<Vector<f32>, LinalgError>;
    /// This method is used to set all components of a quaternion.
    fn fillq(self: &mut Self, qw: f32, qx: f32, qy: f32, qz: f32) -> Result<(), LinalgError>;
    /// This method is used to fill a quaternion as identity quaternion.
    fn fill_identity(self: &mut Self) -> Result<(), LinalgError>;
    /// This method is used to conjugate a quaternion.
    fn conjugate(self: &mut Self) -> Result<(), LinalgError>;
    /// This method is used to create the conjugate of a quaternion.
    fn conjugate_new(self: &Self) -> Result<Vector<f32>, LinalgError>;
    /// This method is used to perform Hamilton product between two quaternions.
    fn mul(self: &mut Self, quat1: &Self, quat2: &Self) -> Result<(), LinalgError>;
    /// This method is used to ...
    fn mul_new(self: &Self, other: &Self) -> Result<Vector<f32>, LinalgError>;
    /// This method is used to invert a quaternion.
    fn invert(self: &mut Self) -> Result<(), LinalgError>;
    /// This method is used to convert a quaternion into a Direct Cosine Matrix (DCM).
    fn convert_to_dcm(self: &Self) -> Result<Matrix, LinalgError>;
    /// This method is used to ...
    fn exp(self: &mut Self) -> Result<(), LinalgError>;
    fn exp_new(self: &Self) -> Result<Vector<f32>, LinalgError>;
    //fn log(self: &mut Self) -> Result<(), LinalgError>;
    //fn mult_L(self: &Self) -> Result<Matrix, LinalgError>;
    //fn mult_R(self: &Self) -> Result<Matrix, LinalgError>;

    /// This method is used to rotate a vector of size 3 x 1 using quaternion.
    fn rotate(self: &Self, vect: &Self) -> Result<Vector<f32>, LinalgError>;
    fn rotate_in_place(self: &Self, vect: &mut Self) -> Result<(), LinalgError>;

    /// This method is used to convert a DCM into a quaternion using Hughe's method.
    /// Source: Hughes, Peter C. Spacecraft Attitude Dynamics. 1th ed. Mineola, New York: Dover Publications Inc., 1986, p. 18.
    fn hughes(self: &mut Self, dcm: &Matrix) -> Result<(), LinalgError>;
    fn chiaverini(self: &mut Self, dcm: &Matrix) -> Result<(), LinalgError>;
    /// This method is used to convert a quaternion into Tait-Bryan (Euler) angles.
    //fn convert_to_euler(self: &mut Self) -> Result<Vector<f32>, LinalgError>;
    fn convert_to_euler(self: &mut Self) -> Result<(f32, f32, f32), LinalgError>;
    // This method is used to convert Tait-Bryan (Euler) angles into a quaternion.
    //fn convert_from_euler(self: &mut Self, euler_angles: &Self) -> Result<(), LinalgError>;
    //fn convert_from_euler(self: &mut Self, phi: f32, theta: f32, psi: f32) -> Result<(), LinalgError>;
}

impl Quaternion for Vector<f32> {
    // This method is used to check if a vector can be used as a quaternion.
    fn is_quaternion(self: &Self) -> Result<bool, LinalgError> {
        // Check that the vector size is 4x1.
        if self.get_rows()? != 4 {
            Ok(false)   // Return false with no error.
        } else {
            Ok(true)    // Return true with no error.
        }
    }

    // This method is used to check if a quaternion is real or not within a specified tolerance.
    // A real quaternion has all elements of its vector part equal to zero: q = w + 0i + 0j + 0k.
    fn is_real(self: &Self) -> Result<bool, LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        // Check if the components of the scalar part are equal to 0.
        for row in 1..4 {
            if !allclose(self.get_element(row)?, 0.0, EPSILON).map_err(LinalgError::UtilsError)? {
                return Ok(false);   // Return false wwith no error.
            }
        }

        Ok(true)    // Return true with no error.
    }

    // This method is used to check if a quaternion is pure or not within a specified tolerance.
    // A pure quaternion has a scalar part equal to zero: q = 0 + xi + yj + zk.
    fn is_pure(self: &Self) -> Result<bool, LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        // Check if the real part component of the quaternion is equal to 0.
        if !allclose(self.get_element(0)?, 0.0, EPSILON).map_err(LinalgError::UtilsError)? {
            return Ok(false);   // Return false wwith no error.
        }

        Ok(true)    // Return true with no error.
    }

    // This method is used to check if a quaternion is a versor or not within a specified tolerance.
    fn is_versor(self: &Self) -> Result<bool, LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        // Check if the norm of the quaternion is equal to 1.
        if !allclose(self.calculate_norm()?, 1.0, EPSILON).map_err(LinalgError::UtilsError)? {
            return Ok(false);   // Return false wwith no error.
        }

        Ok(true)    // Return true with no error.
    }

    // This method is used to verify if a quaternion is the identity quaternion.
    // An identity quaternion has its scalar part equal to 1, and its vector part equal to 0, such that: q = 1 + 0i + 0j + 0k.
    fn is_identity(self: &Self) -> Result<bool, LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        // Check if the real part component of the quaternion is equal to 1.
        if !allclose(self.get_element(0)?, 1.0, EPSILON).map_err(LinalgError::UtilsError)? {
            return Ok(false);   // Return false wwith no error.
        }

        // Check if the components of the scalar part are equal to 0.
        for row in 1..4 {
            if !allclose(self.get_element(row)?, 0.0, EPSILON).map_err(LinalgError::UtilsError)? {
                return Ok(false);   // Return false wwith no error.
            }
        }

        Ok(true)    // Return true with no error.
    }

    // This method is used to set the qw value of a quaternion.
    fn set_qw(self: &mut Self, qw: f32) -> Result<(), LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        self.set_element(0, qw)?;   // Set the qw value.

        Ok(())  // Return no error.
    }

    // This method is used to retrieve the qw value of a quaternion.
    fn get_qw(self: &Self) -> Result<f32, LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        Ok(self.get_element(0)?)
    }

    // This method is used to set the qx value of a quaternion.
    fn set_qx(self: &mut Self, qx: f32) -> Result<(), LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        self.set_element(1, qx)?;   // Set the qx value.

        Ok(())  // Return no error.
    }

    // This method is used to retrieve the qx value of a quaternion.
    fn get_qx(self: &Self) -> Result<f32, LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        Ok(self.get_element(1)?)
    }

    // This method is used to set the qy value of a quaternion.
    fn set_qy(self: &mut Self, qy: f32) -> Result<(), LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        self.set_element(2, qy)?;   // Set the qy value.

        Ok(())  // Return no error.
    }

    // This method is used to retrieve the qy value of a quaternion.
    fn get_qy(self: &Self) -> Result<f32, LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        Ok(self.get_element(2)?)
    }

    // This method is used to set the qz value of a quaternion.
    fn set_qz(self: &mut Self, qz: f32) -> Result<(), LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        self.set_element(3, qz)?;   // Set the qz value.

        Ok(())  // Return no error.
    }

    // This method is used to retrieve the qz value of a quaternion.
    fn get_qz(self: &Self) -> Result<f32, LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        Ok(self.get_element(3)?)
    }

    // This method is used to retrieve the vector part of a quaternion.
    fn get_vector_part(self: &Self) -> Result<Vector<f32>, LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
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

    // This method is used to fill a quaternion.
    fn fillq(self: &mut Self, qw: f32, qx: f32, qy: f32, qz: f32) -> Result<(), LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        // Fill the quaternion.
        self.set_element(0, qw)?;
        self.set_element(1, qx)?;
        self.set_element(2, qy)?;
        self.set_element(3, qz)?;

        Ok(())  // Return no error.
    }

    // This method is used to conjugate a quaternion.
    fn conjugate(self: &mut Self) -> Result<(), LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
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

    fn conjugate_new(self: &Self) -> Result<Vector<f32>, LinalgError> {
        let mut conj = self.duplicate()?;
        conj.conjugate()?;

        Ok(conj)
    }

    // This method is used to perform the multiplication operation (Hamilton product) of two quaternions.
    fn mul(self: &mut Self, quat1: &Self, quat2: &Self) -> Result<(), LinalgError> {
        // Check that these are all quaternions.
        if !self.is_quaternion()? || !quat1.is_quaternion()? || !quat2.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
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

    fn mul_new(self: &Self, other: &Self) -> Result<Vector<f32>, LinalgError> {
        // Check that these are all quaternions.
        if !self.is_quaternion()? || !other.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        // Extract components of the first quaternion (quat1).
        let qw1 = self.get_element(0)?;
        let qx1 = self.get_element(1)?;
        let qy1 = self.get_element(2)?;
        let qz1 = self.get_element(3)?;

        // Extract components of the second quaternion (quat2).
        let qw2 = other.get_element(0)?;
        let qx2 = other.get_element(1)?;
        let qy2 = other.get_element(2)?;
        let qz2 = other.get_element(3)?;

        // Calculate the components of the resulting quaternion (self).
        let qw = qw1*qw2 - qx1*qx2 - qy1*qy2 - qz1*qz2;
        let qx = qw1*qx2 + qx1*qw2 - qz1*qy2 + qy1*qz2;
        let qy = qw1*qy2 + qz1*qx2 + qw2*qy1 - qx1*qz2;
        let qz = qw1*qz2 - qy1*qx2 + qx1*qy2 + qw2*qz1;

        let mut result_quat = Self::new();
        result_quat.init(4)?;

        // Store the calculated components in the resulting quaternion.
        result_quat.set_element(0, qw)?;
        result_quat.set_element(1, qx)?;
        result_quat.set_element(2, qy)?;
        result_quat.set_element(3, qz)?;

        Ok(result_quat) // Return no error.
    }

    // This method is used to fill a quaternion as identity quaternion.
    fn fill_identity(self: &mut Self) -> Result<(), LinalgError> {
        self.fillq(1.0, 0.0, 0.0, 0.0)?;

        Ok(())  // Return no error.
    }

    // This method is used to invert a quaternion.
    fn invert(self: &mut Self) -> Result<(), LinalgError> {
        let norm = self.calculate_norm()?;  // Calculate the norm of the quaternion.
        self.conjugate()?;                  // Conjugate the quaternion.

        if norm > EPSILON {
            self.mul_by_scalar_in_place(1.0_f32 / (norm * norm))?;
        } else {
            self.fillq(0.0, 0.0, 0.0, 0.0)?;
        }

        Ok(())  // Return no error.
    }

    // This method is used to calculate the rotation matrix (Direct Cosine Matrix - DCM) from a quaternion.
    fn convert_to_dcm(self: &Self) -> Result<Matrix, LinalgError> {
        let mut dcm = Matrix::new();    // Create the rotation matrix.
        dcm.init(3, 3)?;                // Initialize it.

        let qw = self.get_qw()?;        // Retrieve qw component from the quaternion.
        let qx = self.get_qx()?;        // Retrieve qx component from the quaternion.
        let qy = self.get_qy()?;        // Retrieve qy component from the quaternion.
        let qz = self.get_qz()?;        // Retrieve qz component from the quaternion.

        let mut element: f32;           // Create a temporary variable to store an element of the matrix.

        // Fill the first column of the rotation matrix.
        //element = 1.0 - 2.0*(qy * qy + qz * qz);
        element = qw*qw + qx*qx - qy*qy - qz*qz;
        dcm.set_element(0, 0, element)?;
        element = 2.0*(qx*qy - qw*qz);
        dcm.set_element(0, 1, element)?;
        element = 2.0 * (qx*qz + qw*qy);
        dcm.set_element(0, 2, element)?;

        element = 2.0*(qx*qy + qw*qz);
        dcm.set_element(1, 0, element)?;
        element = 1.0 - 2.0*(qx*qx + qz*qz);
        dcm.set_element(1, 1, element)?;
        element = 2.0*(qy*qz - qw*qx);
        dcm.set_element(1, 2, element)?;

        element = 2.0*(qx*qz - qw*qy);
        dcm.set_element(2, 0, element)?;
        element = 2.0*(qw*qx + qy*qz);
        dcm.set_element(2, 1, element)?;
        element = 1.0 - 2.0*(qx*qx + qy*qy);
        dcm.set_element(2, 2, element)?;

        Ok(dcm) // Return the rotation matrix with no error.
    }

    // This method is used to ...
    /*
    The quaternion exponential works as in the ordinary case, defined with the absolute convergent power series:
    \[e^{\mathbf{q}} = \sum_{k=0}^{\infty} \frac{\mathbf{q}^k}{k!}\]

    The exponential of a pure quaternion is, with the help of Euler's formula and the series of \(\cos\theta\) and \(\sin\theta\), redefined as:

    // Add some comments here.
    */
    fn exp(self: &mut Self) -> Result<(), LinalgError> {
        // Check if the quaternion is real.
        if self.is_real()? {
            self.fillq(1.0, 0.0, 0.0, 0.0)?;

            return Ok(());  // Return no error.
        }

        let qw = self.get_element(0)?;          // Retrieve real part of the quaternion.
        let mut qv = self.get_vector_part()?;   // Retrieve vector part of the quaternion.
        let qv_norm = qv.calculate_norm()?;     // Calculate its norm.
        qv.normalize()?;                        // Normalize it.

        qv.mul_by_scalar_in_place(sinf(qv_norm))?;

        self.fillq(cosf(qv_norm), qv.get_element(0)?, qv.get_element(1)?, qv.get_element(2)?)?;

        // Check if the quaternion is pure.
        if self.is_pure()? {
            return Ok(());  // Return no error.
        }

        self.mul_by_scalar_in_place(powf(E, qw))?;

        Ok(())  // Return no error.
    }

    // This method is used to.
    fn exp_new(self: &Self) -> Result<Vector<f32>, LinalgError> {
        let mut q_exp = self.duplicate()?;

        // Check if the quaternion is real.
        if self.is_real()? {
            q_exp.fillq(1.0, 0.0, 0.0, 0.0)?;

            return Ok(q_exp);   // Return the result with no error.
        }

        let qw = self.get_element(0)?;          // Retrieve real part of the quaternion.
        let mut qv = self.get_vector_part()?;   // Retrieve vector part of the quaternion.
        let qv_norm = qv.calculate_norm()?;     // Calculate its norm.
        qv.normalize()?;                        // Normalize it.

        qv.mul_by_scalar_in_place(sinf(qv_norm))?;

        q_exp.fillq(cosf(qv_norm), qv.get_element(0)?, qv.get_element(1)?, qv.get_element(2)?)?;

        // Check if the quaternion is pure.
        if self.is_pure()? {
            return Ok(q_exp);   // Return the result with no error.
        }

        q_exp.mul_by_scalar_in_place(powf(E, qw))?;

        Ok(q_exp)   // Return the result with no error.
    }

    // This method is used to convert a DCM into a quaternion using Hughes's method.
    // Peter C. Hughes, Spacecraft Attitude Dynamics. 1th ed. Mineola, New York: Dover Publications Inc., 1986, p. 18.
    // Caution! Here we assume that the matrix passed as a parameter to the method is a rotation matrix. If this is not the case, the algorithm will calculate aberrant results.
    fn hughes(self: &mut Self, dcm: &Matrix) -> Result<(), LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        /*
        // Caution! Very costly in computing resources.
        if !dcm.is_orthogonal()? {
            return Err(LinalgError::NotDCM);    // Return an error.
        }
        */

        let mut trace = dcm.trace()?;
        trace = clip(trace, -1.0, 3.0);

        // Check if the rotation is null.
        if allclose(trace, 3.0, EPSILON).map_err(LinalgError::UtilsError)? {
            // The trace is 3, which implies that the DCM is the identity matrix and therefore the rotation is null.
            self.fillq(1.0, 0.0, 0.0, 0.0)?;

            return Ok(());  // Return no error.
        }

        // Calculate the real part of the quaternion.
        let n = 0.5 * sqrtf(1.0 + trace);  // eq. 15 on p. 18 of "Spacecraft Attitude Dynamics".

        let qx: f32;
        let qy: f32;
        let qz: f32;

        // Calculate the vector part of the quaternion.
        if allclose(n, 0.0, EPSILON).map_err(LinalgError::UtilsError)? {
            // The trace is -1, which implies that the real part of quaternion is null and therefore the quaternion is pure.
            let mut e = dcm.diag(None)?;
            e.add_scalar_in_place(1.0)?;
            e.mul_by_scalar_in_place(0.5)?;
            e.power_elements(0.5)?;

            qx = e.get_element(0)?;
            qy = e.get_element(1)?;
            qz = e.get_element(2)?;
        } else {    // eq. 16 on p. 18 of "Spacecraft Attitude Dynamics".
            qx = 0.25 * (dcm.get_element(1, 2)? - dcm.get_element(2, 1)?) / n;
            qy = 0.25 * (dcm.get_element(2, 0)? - dcm.get_element(0, 2)?) / n;
            qz = 0.25 * (dcm.get_element(0, 1)? - dcm.get_element(1, 0)?) / n;
        }

        // Fill the quaternion.
        self.set_element(0, n)?;
        self.set_element(1, qx)?;
        self.set_element(2, qy)?;
        self.set_element(3, qz)?;

        Ok(())  // Return no error.
    }

    // This method is used to convert a DCM into a quaternion using Chiaverini's method.
    // S. Chiaverini & B. Siciliano, The Unit Quaternion: A Useful Tool for Inverse Kinematics of Robot Manipulators. OPA, Overseas Publishers Association, 1999.
    // Caution! Here we assume that the matrix passed as a parameter to the method is a rotation matrix. If this is not the case, the algorithm will calculate aberrant results.
    fn chiaverini(self: &mut Self, dcm: &Matrix) -> Result<(), LinalgError> {
        // Check that it is a quaternion.
        if !self.is_quaternion()? {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        /*
        // Caution! Very costly in computing resources.
        if !dcm.is_orthogonal()? {
            return Err(LinalgError::NotDCM);    // Return an error.
        }
        */

        let mut trace = dcm.trace()?;
        trace = clip(trace, -1.0, 3.0);

        // TODO: try to be more efficient.
        let dcm_32 = dcm.get_element(2, 1)?;
        let dcm_23 = dcm.get_element(1, 2)?;
        let dcm_11 = dcm.get_element(0, 0)?;
        let dcm_22 = dcm.get_element(1, 1)?;
        let dcm_33 = dcm.get_element(2, 2)?;
        let dcm_13 = dcm.get_element(0, 2)?;
        let dcm_31 = dcm.get_element(2, 0)?;
        let dcm_21 = dcm.get_element(1, 0)?;
        let dcm_12 = dcm.get_element(0, 1)?;

        let qw = 0.5 * sqrtf(trace + 1.0);
        let qx = 0.5 * (dcm_32 - dcm_23).sign() * sqrtf(clip(dcm_11 - dcm_22 - dcm_33, -1.0, 1.0) + 1.0);
        let qy = 0.5 * (dcm_13 - dcm_31).sign() * sqrtf(clip(dcm_22 - dcm_33 - dcm_11, -1.0, 1.0) + 1.0);
        let qz = 0.5 * (dcm_21 - dcm_12).sign() * sqrtf(clip(dcm_33 - dcm_11 - dcm_22, -1.0, 1.0) + 1.0);

        // Fill the quaternion.
        self.set_element(0, qw)?;
        self.set_element(1, qx)?;
        self.set_element(2, qy)?;
        self.set_element(3, qz)?;

        // TODO: check if the quaternion is correct. ex: (0 0 0 0) is not force qw to 1 or return an error.
        // Add some code here.

        self.normalize()?;

        Ok(())  // Return no error.
    }

    // This method is used to convert a quaternion into Tait-Bryan (Euler) angles.
    //fn convert_to_euler(self: &mut Self) -> Result<Vector<f32>, LinalgError> {
    fn convert_to_euler(self: &mut Self) -> Result<(f32, f32, f32), LinalgError> {
        /*
        let mut euler_angles: Vector<f32> = Vector::new();  // Create a new vector to store calculated Euler angles.
        euler_angles.init(3)?;                              // Initialize it.
        */

        // Extract each component of the quaternion.
        let qw = self.get_qw()?;
        let qx = self.get_qx()?;
        let qy = self.get_qy()?;
        let qz = self.get_qz()?;

        let phi = atan2f(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy));   // Calculate roll angle.
        let theta = asinf(2.0*(qw*qy - qz*qx));                             // Calculate pitch angle.
        let psi = atan2f(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));   // Calculate yaw angle.

        // Store each angle in the vector.
        /*
        euler_angles.set_element(0, phi)?;
        euler_angles.set_element(1, theta)?;
        euler_angles.set_element(2, psi)?;
        */

        //Ok(euler_angles)        // Return Euler angles with no error.
        Ok((phi, theta, psi))   // Return Euler angles with no error.
    }

    // This method is used to convert Tait-Bryan (Euler) angles into a quaternion.
    /*
    //fn convert_from_euler(self: &mut Self, euler_angles: &Self) -> Result<(), LinalgError>;
    fn convert_from_euler(self: &mut Self, phi: f32, theta: f32, psi: f32) -> Result<(), LinalgError> {

        // Add some code here.

        Ok(())  // Return no error.
    }
    */

    // This method is used to rotate a vector of size 3 x 1 using quaternion.
    fn rotate(self: &Self, vect: &Self) -> Result<Vector<f32>, LinalgError> {
        // Check that the size of the vector is 3 x 1.
        if vect.get_rows()? != 3 {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        // Not very computationally efficient. Quaternions can be implicitly converted to a rotation-like matrix (12 multiplications and 12 additions/subtractions), which levels the following vectors rotating cost with the rotation matrix method.
        let dcm = self.convert_to_dcm()?;
        let rotated_vect = dcm.mul_new(&vect.convert_to_matrix()?)?;

        let rotated_vect = rotated_vect.col_to_vector(0)?;

        Ok(rotated_vect)    // Return the rotated vector with no error.

        /*
        let vect_quat: Vector<f32> = Vector::new();
        vect_quat.init(4)?;

        vect_quat.set_qw(0.0)?;
        vect_quat.set_qx(vect.get_element(0)?)?;
        vect_quat.set_qy(vect.get_element(1)?)?;
        vect_quat.set_qz(vect.get_element(2)?)?;
        */
    }

    fn rotate_in_place(self: &Self, vect: &mut Self) -> Result<(), LinalgError> {
        // Check that the size of the vector is 3 x 1.
        if vect.get_rows()? != 3 {
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        let dcm = self.convert_to_dcm()?;
        let temp = dcm.mul_new(&vect.convert_to_matrix()?)?;
        temp.get_col(vect, 0)?;

        Ok(())  // Return no error.
    }

    //fn differentiation(self: &Self, other: &Self)
}
