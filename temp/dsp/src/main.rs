extern crate utils;
extern crate libm;

use utils::utils::{is_valid_value, allclose, UtilsError};
use libm::{cosf, sinf, sqrtf, expf};

pub const EPSILON: f32 = 0.0001;        /// Constant value representing a minimal error in numerical analysis.
pub const N_MAX: usize = 16;            /// Maximum sample size (number of elements).
pub const PI: f32 = 3.141592653589793;  /// Approximate value of constant π.

//#[derive(Debug)]
#[derive(Debug, Copy, Clone)]
/// Complex number structure.
pub struct Complex {
    re: f32,    // Real (scalar or Re(Z)) part of the complex number.
    im: f32,    // Imaginary (Im(Z)) part of the complex number.
}

impl Complex {
    /// This method is used to create a new complex number (rectangular form).
    pub fn new(re: f32, im: f32) -> Result<Self, UtilsError> {
        is_valid_value(re)?;    // Check that real part is valid.
        is_valid_value(im)?;    // Check that imaginary part is valid.

        let z = Self {
            re: re, // Set real part of the complex number.
            im: im, // Set imaginary part of the complex number.
        };

        Ok(z)   // Return no error.
    }

    /// This method is used to set the real part of a complex number.
    pub fn set_real_part(self: &mut Self, re: f32) -> Result<(), UtilsError> {
        is_valid_value(re)?;    // Check that the real part of the complex number is valid.

        self.re = re;   // Set the real part of the complex number.

        Ok(())  // Return no error.
    }

    /// This method is used to retrieve the real part of a complex number.
    pub fn get_real_part(self: &Self) -> Result<f32, UtilsError> {
        is_valid_value(self.re)?;   // Check that the real part of the complex number is valid.

        Ok(self.re) // Return the real part of the complex number with no error.
    }

    /// This method is used to set the imaginary part of a complex number.
    pub fn set_imaginary_part(self: &mut Self, im: f32) -> Result<(), UtilsError> {
        is_valid_value(im)?;    // Check that the imaginary part is valid.

        self.im = im;   // Set the imaginary part of the complex number.

        Ok(())  // Return no error.
    }

    /// This method is used to retrieve the imaginary part of a complex number.
    pub fn get_imaginary_part(self: &Self) -> Result<f32, UtilsError> {
        is_valid_value(self.im)?;   // Check that the imaginary part of the complex number is valid.

        Ok(self.im) // Return the imaginary part of the complex number with no error.
    }

    /// This method is used to copy a complex number.
    pub fn copy_from(self: &mut Self, other: &Self) -> Result<(), UtilsError> {
        let re = other.get_real_part()?;        // Retrieve real part of the reference complex number.
        let im = other.get_imaginary_part()?;   // Retrieve imaginary part of the reference complex number.

        self.set_real_part(re)?;        // Set the real part of the complex number.
        self.set_imaginary_part(im)?;   // Set the imaginary part of the complex number.

        Ok(())  // Return no error.
    }

    /// This method is used to duplicate a complex number.
    pub fn duplicate(self: &Self) -> Result<Self, UtilsError> {
        let mut duplicated_complex_number = Self::new(0.0, 0.0)?;   // Create a new complex number.

        duplicated_complex_number.copy_from(&self)?;    // Copy real and imaginary parts of the original one.

        Ok(duplicated_complex_number)   // Return the duplicated complex number with no error.
    }

    /// This method is used to convert polar form of a complex number into rectangular form.
    pub fn convert_from_polar(self: &mut Self, r: f32, theta: f32) -> Result<(), UtilsError> {
        is_valid_value(r)?;     // Check that the module is valid.
        is_valid_value(theta)?; // Check that the argument is valid.

        let re = r * cosf(theta);   // Calculate the real part of the complex number.
        let im = r * sinf(theta);   // Calculate the imaginary part of the complex number.

        self.set_real_part(re)?;        // Set the real part of the complex number.
        self.set_imaginary_part(im)?;   // Set the imaginary part of the complex number.

        Ok(()) // Return no error.
    }

    /// This method is used to ...
    pub fn neg(self: &mut Self) -> Result<(), UtilsError> {
        let re = self.get_real_part()?;         // Retrieve real part of the complex number.
        let im = self.get_imaginary_part()?;    // Retrieve imaginary part of the complex number.

        self.set_real_part(-re)?;       // Set the real part of the complex number.
        self.set_imaginary_part(-im)?;  // Set the imaginary part of the complex number.

        Ok(())  // Return no error.
    }

    /// This method is used to conjguate a complex number.
    pub fn conjugate(self: &mut Self) -> Result<(), UtilsError> {
        let im = self.get_imaginary_part()?;    // Retrieve imaginary part of the complex number.

        self.set_imaginary_part(-im)?;  // Set the imaginary part of the complex number.

        Ok(())  // Return no error.
    }

    /// This method is used to calculate the module of a complex number.
    pub fn calculate_module(self: &mut Self) -> Result<f32, UtilsError> {
        let re = self.get_real_part()?;         // Retrieve real part.
        let im = self.get_imaginary_part()?;    // Retrieve imaginary part.

        let mut module = sqrtf(re * re + im * im);  // Calculate the module of the complex number.

        // Check that the module is not too close to zero. If it is, round it down to zero.
        if allclose(module, 0.0, EPSILON)? {
            module = 0.0;   // Set the module to zero.
        }

        Ok(module)  // Return the module of the complex number with no error.
    }

    /// This method is used to ...
    pub fn expo(self: &mut Self) -> Result<(), UtilsError> {
        let re = self.get_real_part()?; // Retrieve real part of the complex number.

        let m = expf(re);   // Calculate ...

        if allclose(m, 0.0, EPSILON)? {
            self.re = 0.0;  // Set the real part to zero.
            self.im = 0.0;  // Set the imaginary part to zero.
        } else {
            self.re = m * cosf(self.im);    // ...
            self.im = m * sinf(self.im);    // ...
        }

        Ok(())  // Return no error.
    }

    /// This method is used to perform the addition of two complex number and store the result into another one.
    pub fn add(self: &mut Self, z1: &Self, z2: &Self) -> Result<(), UtilsError> {
        self.re = z1.get_real_part()? + z2.get_real_part()?;            // Set the real part of the complex number.
        self.im = z1.get_imaginary_part()? + z2.get_imaginary_part()?;  // Set the imaginary part of the complex number.

        Ok(())  // Return no error.
    }

    /// This method is used to ...
    pub fn add_in_place(self: &mut Self, other: &Self) -> Result<(), UtilsError> {
        self.re += other.get_real_part()?;      // ...
        self.im += other.get_imaginary_part()?; // ...

        Ok(())  // Return no error.
    }

    /// This method is used to perform the subtraction of two complex number and store the result into another one.
    pub fn sub(self: &mut Self, z1: &Self, z2: &Self) -> Result<(), UtilsError> {
        self.re = z1.get_real_part()? - z2.get_real_part()?;            // Set the real part of the complex number.
        self.im = z1.get_imaginary_part()? - z2.get_imaginary_part()?;  // Set the imaginary part of the complex number.

        Ok(())  // Return no error.
    }

    /// This method is used to ...
    pub fn sub_in_place(self: &mut Self, other: &Self) -> Result<(), UtilsError> {
        self.re -= other.get_real_part()?;      // ...
        self.im -= other.get_imaginary_part()?; // ...

        Ok(())  // Return no error.
    }

    /// This method is used to perform the multiplication of two complex number and store the result into another one.
    pub fn mul(self: &mut Self, z1: &Self, z2: &Self) -> Result<(), UtilsError> {
        let z1_re = z1.get_real_part()?;    // Retrieve real part of the first complex number.
        let z2_re = z2.get_real_part()?;    // Retrieve real part of the second complex number.

        let z1_im = z1.get_imaginary_part()?;   // Retrieve imaginary part of the first complex number.
        let z2_im = z2.get_imaginary_part()?;   // Retrieve imaginary part of the second complex number.

        self.re = z1_re * z2_re - z1_im * z2_im;    // Set real part of the complex number.
        self.im = z1_re * z2_im + z1_im * z2_re;    // Set imaginary part of the complex number.

        Ok(())  // Return no error.
    }

    /// This method is used to ...
    pub fn mul_in_place(self: &mut Self, other: &Self) -> Result<(), UtilsError> {
        let re_temp = self.get_real_part()?;        // Retrieve real part of the complex number.
        let im_temp = self.get_imaginary_part()?;   // Retrieve imaginary part of the complex number.

        let other_re = other.get_real_part()?;      // Retrieve real part of the other complex number.
        let other_im = other.get_imaginary_part()?; // Retrieve imaginary part of the other complex number.

        self.re = re_temp * other_re - im_temp * other_im;  // Set real part of the complex number.
        self.im = re_temp * other_im + im_temp * other_re;  // Set imaginary part of the complex number.

        Ok(())  // Return no error.
    }

    /// This method is used to perform the division of two complex numbers and store the result into another one.
    pub fn div(self: &mut Self, z1: &Self, z2: &Self) -> Result<(), UtilsError> {
        let z1_re = z1.get_real_part()?;    // Retrieve real part of the first complex number.
        let z2_re = z2.get_real_part()?;    // Retrieve real part of the second complex number.

        let z1_im = z1.get_imaginary_part()?;   // Retrieve imaginary part of the first complex number.
        let z2_im = z2.get_imaginary_part()?;   // Retrieve imaginary part of the second complex number.

        // Check that the denominator is not zero.
        if allclose(z2_re, 0.0, EPSILON)? && allclose(z2_im, 0.0, EPSILON)? {
            return Err(UtilsError::Inf);
        }

        let denominator = z2_re * z2_re + z2_im * z2_im;    // Calculate the denominator.

        self.re = (z1_re * z2_re + z1_im * z2_im) / denominator;    // Set real part of the complex number.
        self.im = (z1_im * z2_re - z1_re * z2_im) / denominator;    // Set imaginary part of the complex number.

        Ok(())  // Return no error.
    }

    /// This method is used to swap two complex numbers.
    pub fn swap(self: &mut Self, other: &mut Self) -> Result<(), UtilsError> {
        let mut temp = self.get_real_part()?;   // Retrieve real part of the complex number.
        self.re = other.get_real_part()?;       // ...
        other.re = temp;                        // ...

        temp = self.get_imaginary_part()?;      // Retrieve imaginary part of the complex number.
        self.im = other.get_imaginary_part()?;  // ...
        other.im = temp;                        // ...

        Ok(())  // Return no error.
    }
}

#[allow(non_snake_case)]
fn dft_naive(x: [Complex; N_MAX]) -> Result<[Complex; N_MAX], UtilsError> {
    let mut X: [Complex; N_MAX] = [Complex::new(0.0, 0.0)?; N_MAX];

    for k in 0..N_MAX {
        let mut sum = Complex::new(0.0, 0.0)?;

        for n in 0..N_MAX {
            let angle = -2.0 * PI * (k as f32) * (n as f32) / (N_MAX as f32);
            let w = Complex::new(cosf(angle), sinf(angle))?;
            //let mut temp = x[n];
            let mut temp = x[n].duplicate()?;
            temp.mul_in_place(&w)?;
            sum.add_in_place(&temp)?;
        }

        //X[k] = sum;
        X[k].copy_from(&sum)?;
    }

    Ok(X)   // Return the result of the DFT with no error.
}

#[allow(non_snake_case)]
fn main() {
    // Création d'un tableau d'exemple de nombres complexes
    let x: [Complex; N_MAX] = [
        Complex::new(1.0, 0.0).unwrap(),
        Complex::new(2.0, 0.0).unwrap(),
        Complex::new(3.0, 0.0).unwrap(),
        Complex::new(4.0, 0.0).unwrap(),
        Complex::new(1.0, 0.0).unwrap(),
        Complex::new(2.0, 0.0).unwrap(),
        Complex::new(3.0, 0.0).unwrap(),
        Complex::new(4.0, 0.0).unwrap(),
        Complex::new(1.0, 0.0).unwrap(),
        Complex::new(2.0, 0.0).unwrap(),
        Complex::new(3.0, 0.0).unwrap(),
        Complex::new(4.0, 0.0).unwrap(),
        Complex::new(1.0, 0.0).unwrap(),
        Complex::new(2.0, 0.0).unwrap(),
        Complex::new(3.0, 0.0).unwrap(),
        Complex::new(4.0, 0.0).unwrap(),
    ];

    // Calculate the DFT of the discrete signal with naive method.
    match dft_naive(x) {
        Ok(X) => {
            // Print the result.
            for (i, x) in X.iter().enumerate() {
                println!("X[{}] = {:.2} + {:.2}i", i, x.get_real_part().unwrap(), x.get_imaginary_part().unwrap());
            }
        }
        Err(_) => println!("An error occured while performing the DFT."),
    }
}
