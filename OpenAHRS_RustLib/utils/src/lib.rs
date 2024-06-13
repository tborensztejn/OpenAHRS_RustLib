#![no_std]

pub mod utils {
    use libm::fabsf;

    #[derive(Debug)]
    #[derive(PartialEq)]
    pub enum UtilsError {
        Nan,
        Inf,
    }

    pub fn is_valid_value(value: f32) -> Result<(), UtilsError> {
        if value.is_nan() {
            Err(UtilsError::Nan)
        } else if value.is_infinite() {
            Err(UtilsError::Inf)
        } else {
            Ok(())
        }
    }

    pub fn close_all(x: f32, y: f32, deviation: f32) -> Result<bool, UtilsError> {
        if x.is_nan() || y.is_nan() {
            Err(UtilsError::Nan)
        } else if x.is_infinite() || y.is_infinite() {
            Err(UtilsError::Inf)
        } else {
            let result = fabsf(x - y);

            Ok(result <= deviation)
        }
    }

    pub fn in_range<T: PartialOrd + Sized>(value: T, start: T, end: T) -> bool {
        if value >= start && value <= end {
            true
        } else {
            false
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
