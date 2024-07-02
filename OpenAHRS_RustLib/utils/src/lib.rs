#![no_std]

pub mod utils {
    use core::cmp::PartialOrd;
    use core::ops::Neg;
    use libm::fabsf;

    #[derive(Debug, PartialEq)]
    pub enum UtilsError {
        Nan,
        Inf,
    }

    /// This function is used to check the validity of a value before using it.
    pub fn is_valid_value(value: f32) -> Result<(), UtilsError> {
        if value.is_nan() {
            Err(UtilsError::Nan)    // Return an error.
        } else if value.is_infinite() {
            Err(UtilsError::Inf)     // Return an error.
        } else {
            Ok(())   // Return no error.
        }
    }

    /// This function is used to check whether two valid values are equal within a specified tolerance.
    pub fn allclose(x: f32, y: f32, deviation: f32) -> Result<bool, UtilsError> {
        is_valid_value(x)?;
        is_valid_value(y)?;

        Ok(fabsf(x - y) <= deviation)   // Return the verification result with no error.
    }

    /// This function is used to check whether a value belongs to a given interval.
    pub fn in_range<T: PartialOrd>(value: T, start: T, end: T) -> bool {
        value >= start && value <= end
    }

    /// This function is used to ...
    pub fn min<T: Ord>(x: T, y: T) -> T {
        if x < y { x } else { y }
    }

    /// This function is used to ...
    pub fn max<T: Ord>(x: T, y: T) -> T {
        if x > y { x } else { y }
    }

    /// This function is used to ...
    pub fn minf<T: PartialOrd>(x: T, y: T) -> T {
        if x < y { x } else { y }
    }

    /// This function is used to ...
    pub fn maxf<T: PartialOrd>(x: T, y: T) -> T {
        if x > y { x } else { y }
    }

    /// This function is used to ...
    pub fn clip<T: PartialOrd>(value: T, min: T, max: T) -> T {
        if value < min {
            min
        } else if value > max {
            max
        } else {
            value
        }
    }

    /// This function is used to calculate the factorial of a natural number.
    pub fn factorial(n: u8) -> u64 {
        match n {
            0 | 1 => 1,                         // Check for base cases: factorial of 0 and 1 is 1.
            _ => n as u64 * factorial(n - 1),   // Recursive call to calculate the factorial for n greater than 1.
        }
    }

    /// Trait to define the sign function for various numeric types.
    pub trait Sign {
        /// This function is used to determine the sign of a value.
        fn sign(&self) -> Self;
    }

    /// Helper trait to convert from i8 to the target type.
    pub trait FromI8 {
        fn from_i8(n: i8) -> Self;
    }

    /// Implement the helper trait for the necessary types.
    impl FromI8 for i32 {
        fn from_i8(n: i8) -> Self {
            n as i32
        }
    }

    impl FromI8 for f32 {
        fn from_i8(n: i8) -> Self {
            n as f32
        }
    }

    impl FromI8 for f64 {
        fn from_i8(n: i8) -> Self {
            n as f64
        }
    }

    /// Implement the Sign trait for all types that implement the necessary traits.
    impl<T> Sign for T
    where
        T: PartialOrd + Neg<Output = T> + Copy + FromI8,
    {
        // This function is used to determine the sign of a value.
        fn sign(&self) -> Self {
            if *self > T::from_i8(0) {
                T::from_i8(1)
            } else if *self < T::from_i8(0) {
                T::from_i8(-1)
            } else {
                T::from_i8(0)
            }
        }
    }
}
