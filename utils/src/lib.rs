#![no_std]

pub mod utils {
    use libm::fabs;

    #[derive(Debug)]
    #[derive(PartialEq)]
    pub enum UtilsError {
        Nan,
        Inf,
    }

    pub fn is_valid_value(value: f64) -> Result<(), UtilsError> {
        if value.is_nan() {
            Err(UtilsError::Nan)
        } else if value.is_infinite() {
            Err(UtilsError::Inf)
        } else {
            Ok(())
        }
    }

    pub fn close_all(x: f64, y: f64, deviation: f64) -> Result<bool, UtilsError> {
        if x.is_nan() || y.is_nan() {
            Err(UtilsError::Nan)
        } else if x.is_infinite() || y.is_infinite() {
            Err(UtilsError::Inf)
        } else {
            let result = fabs(x - y);

            Ok(result <= deviation)
        }
    }

    pub fn min<T: Ord + Sized>(x: T, y: T) -> T {
        if x < y {
            x
        } else {
            y
        }
    }

    pub fn max<T: Ord + Sized>(x: T, y: T) -> T {
        if x > y {
            x
        } else {
            y
        }
    }

    pub fn factorial(n: u8) -> u64 {
        // Initialize the result variable.
        let ret: u64;

        // Check for base cases: factorial of 0 and 1 is 1.
        if n == 0 || n == 1 {
            ret = 1;    // The factorial of 0 and 1 is 1.
        } else {
            // Recursive call to calculate the factorial for n greater than 1.
            ret = n as u64 * factorial(n - 1);
        }

        ret // Return the calculated factorial.
    }
}
