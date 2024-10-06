extern crate utils;
extern crate libm;

use utils::utils::{is_valid_value, allclose, UtilsError};
use libm::{cosf, sinf, sqrtf, expf};

pub const EPSILON: f32 = 0.0001;    /// Constant value representing a minimal error in numerical analysis.

//#[derive(Debug)]
#[derive(Debug, Copy, Clone)]
/// Complex number structure.
pub struct Complex {
    re: f32,    // Real (scalar or Re(Z)) part of the complex number.
    im: f32,    // Imaginary (Im(Z)) part of the complex number.
}

/*
impl Default for Complex {
    fn default() -> Self {
        Self {
            re: 0.0,    // Default real part.
            im: 0.0,    // Default imaginary part.
        }
    }
}
*/

impl Complex {
    /// This method is used to create a new complex number (rectangular form).
    pub fn new(re: f32, im: f32) -> Result<Self, UtilsError> {
        //Self::default()   // Create default complex number

        is_valid_value(re)?;    // Check that the real part value is valid.
        is_valid_value(im)?;    // Check that the imaginary part value is valid.

        let z = Self {
            re: 0.0,    // Default real part value of the complex number.
            im: 0.0,    // Default imaginary part valueof the complex number.
        };

        Ok(z)   // Return no error.
    }

    /// This method is used to set the real part value of a complex number.
    pub fn set_real_part(self: &mut Self, re: f32) -> Result<(), UtilsError> {
        is_valid_value(re)?;    // Check that the real part value is valid.

        self.re = re;   // Set the real part of the complex number.

        Ok(())  // Return no error.
    }

    /// This method is used to retrieve the real part value of a complex number.
    pub fn get_real_part(self: &Self) -> Result<f32, UtilsError> {
        Ok(self.re) // Return the real part value of the complex number with no error.
    }

    /// This method is used to set the imaginary part value of a complex number.
    pub fn set_imaginary_part(self: &mut Self, im: f32) -> Result<(), UtilsError> {
        is_valid_value(im)?;    // Check that the imaginary part value is valid.

        self.im = im;   // Set the imaginary part of the complex number.

        Ok(())  // Return no error.
    }

    /// This method is used to retrieve the imaginary part value of a complex number.
    pub fn get_imaginary_part(self: &Self) -> Result<f32, UtilsError> {
        Ok(self.im) // Return the imaginary part value of the complex number with no error.
    }

    /// This method is used to convert polar form of a complex number into rectangular form.
    pub fn convert_from_polar(self: &mut Self, r: f32, theta: f32) -> Result<(), UtilsError> {
        is_valid_value(r)?;     // Check that the module value is valid.
        is_valid_value(theta)?; // Check that the argument value is valid.

        let re = r * cosf(theta);   // Calculate the real part of the complex number.
        let im = r * sinf(theta);   // Calculate the imaginary part of the complex number.

        self.set_real_part(re)?;        // Set the real part of the complex number.
        self.set_imaginary_part(im)?;   // Set the imaginary part of the complex number.

        Ok(()) // Return no error.
    }

    /// This method is used to ...
    pub fn neg(self: &mut Self) -> Result<(), UtilsError> {
        self.set_real_part(-self.re)?;      // Set the real part of the complex number.
        self.set_imaginary_part(-self.im)?; // Set the imaginary part of the complex number.

        Ok(())  // Return no error.
    }

    /// This method is used to conjguate a complex number.
    pub fn conjugate(self: &mut Self) -> Result<(), UtilsError> {
        self.set_imaginary_part(-self.im)?; // Set the imaginary part of the complex number.

        Ok(())  // Return no error.
    }

    /// This method is used to calculate the module of a complex number.
    pub fn calculate_module(self: &mut Self) -> Result<f32, UtilsError> {
        let mut module = sqrtf(self.re * self.re + self.im * self.im);  // Calculate the module of the complex number.

        if allclose(module, 0.0, EPSILON)? {
            module = 0.0;
        }

        Ok(module)  // Return the module value of the complex number with no error.
    }

    /// This method is used to ...
    pub fn expo(self: &mut Self) -> Result<(), UtilsError> {
        let m = expf(self.re);  // Calculate ...

        if allclose(m, 0.0, EPSILON)? {
            self.re = 0.0;
            self.im = 0.0;
        } else {
            self.re = m * cosf(self.im);
            self.im = m * sinf(self.im);
        }

        Ok(())  // Return no error.
    }

    /// This method is used to perform the addition of two complex number and store the result into another one.
    pub fn add(self: &mut Self, z1: &Self, z2: &Self) -> Result<(), UtilsError> {
        self.re = z1.re + z2.re;    // Set the real part of the complex number.
        self.im = z1.im + z2.im;    // Set the imaginary part of the complex number.

        Ok(())  // Return no error.
    }

    /// This method is used to perform the subtraction of two complex number and store the result into another one.
    pub fn sub(self: &mut Self, z1: &Self, z2: &Self) -> Result<(), UtilsError> {
        self.re = z1.re - z2.re;
        self.im = z1.im - z2.im;

        Ok(())  // Return no error.
    }

    /// This method is used to perform the multiplication of two complex number and store the result into another one.
    pub fn mul(self: &mut Self, z1: &Self, z2: &Self) -> Result<(), UtilsError> {
        self.re = z1.re * z2.re - z1.im * z2.im;
        self.im = z1.re * z2.im + z1.im * z2.re;

        Ok(())  // Return no error.
    }

    /// This method is used to perform the division of two complex numbers and store the result into another one.
    pub fn div(self: &mut Self, z1: &Self, z2: &Self) -> Result<(), UtilsError> {
        // Check that the denominator is not zero.
        if allclose(z2.re, 0.0, EPSILON)? && allclose(z2.im, 0.0, EPSILON)? {
            return Err(UtilsError::Nan);
        }

        let denominator = z2.re * z2.re + z2.im * z2.im;    // Calculate the denominator.

        self.re = (z1.re * z2.re + z1.im * z2.im) / denominator;
        self.im = (z1.im * z2.re - z1.re * z2.im) / denominator;

        Ok(())  // Return no error.
    }

    // Update here.
    pub fn swap(self: &mut Self, other: &mut Self) -> Result<(), UtilsError> {
        let mut temp = self.re;
        self.re = other.re;
        other.re = temp;
        temp = self.im;
        self.im = other.im;
        other.im = temp;

        Ok(())  // Return no error.
    }
}
