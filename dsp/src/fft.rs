extern crate utils;

use utils::utils::UtilsError;

use crate::complex::{EPSILON, Complex};

pub const N_MAX: usize = 16;            /// Maximum sample size (number of elements).
pub const PI: f32 = 3.141592653589793;  /// Approximate value of constant π.

#[derive(Debug, PartialEq)]
/// All FFT errors.
pub enum FFTError {
    BitReverseError,    /// This error occurs when b (the number of bits) exceeds 8, which makes no sense for a variable of type u8, which can only contain up to 8 bits.

    UtilsError(UtilsError),
}

impl From<UtilsError> for FFTError {
    fn from(error: UtilsError) -> Self {
        FFTError::UtilsError(error)
    }
}

#[derive(Debug, Copy, Clone)]
/// FFT structure.
pub struct FFT {
    n: u8,                  // Number of elements.
    elements: [f32; N_MAX], // Linear array containing all elements of the sample.
}

impl FFT {
    /// This method is used to create a new ...
    pub fn new(self: &mut Self) -> Self {
        Self {
            n: N_MAX as u8,         // Default number of elements.
            elements: [0.0; N_MAX], // Default elements value.
        }
    }

    /// This method is use to perform bit reversal of the integer 'n' with 'b' bits.
    fn bitrev(n: u8, b: u8) -> Result<u8, FFTError> {
        // Check that ...
        if b > 8 {
            return Err(FFTError::BitReverseError);  // Return an error.
        }

        let mut r = 0;
        let mut n = n;

        for m in (0..b).rev() {
            if (n >> m) & 1 == 1 {
                r |= 1 << (b - 1 - m);
                n &= !(1 << m);
            }
        }

        Ok(r)   // Return the reversed bit with no error.
    }

    /// This method is used to ...
    fn dftmerge(self: &Self, x: &mut [Complex]) -> Result<(), FFTError> {
        let mut m: u8 = 2;

        while m <= self.n {
            //let mut w = Complex::new(0.0, -2.0 * PI / m as f32).map_err(FFTError::UtilsError)?;
            //let mut v = Complex::new(0.0, 0.0).map_err(FFTError::UtilsError)?;

            let mut w = Complex::new(0.0, -2.0 * PI / m as f32)?;
            let mut v = Complex::new(0.0, 0.0)?;

            w.expo()?;

            for k in 0..(m / 2) {
                for i in (0..self.n).step_by(m as usize) {
                    let p = k + i;
                    let q = p + m / 2;

                    let a = x[p as usize];
                    let mut b = Complex::new(0.0, 0.0)?;
                    b.mul(&x[q as usize], &v)?;
                }

                //v.mul_in_place(&w)?;
            }

            m *= 2;  // Increment counter.
        }

        Ok(())  // Return no error.
    }
}
