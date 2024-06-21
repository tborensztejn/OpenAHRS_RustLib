extern crate utils;

use utils::utils::{is_valid_value, in_range};
use libm::{acosf, sinf, sqrtf, powf};
use crate::common::{M_MAX, EPSILON, LinalgError};
use crate::common::{is_valid_rows_number, is_valid_row};
use crate::matrix::Matrix;

#[derive(Debug)]
/// Vector structure.
pub struct Vector<T> {
    rows: u8,               // Number of rows in the vector.
    elements: [T; M_MAX],   // All the elements of the vector in a linear array.
    initialized: bool,      // This variable is used to identify whether or not a vector has already been initialized. An uninitialized vector cannot be manipulated.
}

impl<T: Default + Copy> Vector<T> {
    // This method is used to create a new vector of size m x 1.
    pub fn new() -> Self {
        Self {
            rows: 0,
            elements: [Default::default(); M_MAX],
            initialized: false,
        }
    }

    // This method is used to initialize a vector of size m x 1.
    pub fn init(self: &mut Self, rows: u8) -> Result<(), LinalgError> {
        // Check if the vector has already been initialized.
        if self.initialized {
            // The vector has already been initialized.
            return Err(LinalgError::AlreadyInit);   // Return an error.
        }

        is_valid_rows_number(rows)?;    // Check that the number of rows does not exceed M_MAX.

        self.rows = rows;               // Set the number of rows in the vector.
        self.initialized = true;        // Set the initialization flag to true.

        Ok(())  // Return no error.
    }

    // This method is used to reinitialize a vector of size m x 1.
    pub fn reinit(self: &mut Self, rows: u8) -> Result<(), LinalgError> {
        // Check that the vector is initialized.
        if !self.initialized {
            // The vector is not initialized.
            return Err(LinalgError::NotInit);   // Return an error.
        }

        self.initialized = false;   // Set the initialization flag to false.

        self.init(rows)?;           // Reinitialize the vector.

        Ok(())  // Return no error.
    }

    pub fn get_rows(self: &Self) -> Result<u8, LinalgError> {
        // Check that the vector is initialized.
        if !self.initialized {
            // The vector is not initialized.
            return Err(LinalgError::NotInit);   // Return an error.
        }

        Ok(self.rows)   // Return the value with no error.
    }

    pub fn is_initialized(self: &Self) -> bool {
        self.initialized
    }
}

impl Vector<u8> {
    // This method is used to assign a value to a specific element of a vector of size m x 1.
    pub fn set_element(self: &mut Self, row: u8, value: u8) -> Result<(), LinalgError> {
        // Check that the vector is initialized.
        if !self.initialized {
            // The vector is not initialized.
            return Err(LinalgError::NotInit);   // Return an error.
        }

        is_valid_row(row, self.rows)?;  // Check if the row exists.

        self.elements[row as usize] = value;    // Set the value to the specified vector element.

        Ok(())  // Return no error.
    }

    // This method is used to access a specific element of a vector of size m x 1.
    pub fn get_element(self: &Self, row: u8) -> Result<u8, LinalgError> {
        // Check that the vector is initialized.
        if !self.initialized {
            // The vector is not initialized.
            return Err(LinalgError::NotInit);   // Return an error.
        }

        is_valid_row(row, self.rows)?;  // Check if the row exists.

        let value = self.elements[row as usize];    // Retrieve the value of the specified element from the vector.

        Ok(value)   // Return the value with no error.
    }
}

impl Vector<f32> {
    /// This method is used to assign a value to a specific element of a vector of size m x 1.
    pub fn set_element(self: &mut Self, row: u8, value: f32) -> Result<(), LinalgError> {
        // Check that the vector is initialized.
        if !self.initialized {
            // The vector is not initialized.
            Err(LinalgError::NotInit)   // Return an error.
        } else {
            is_valid_value(value).map_err(LinalgError::UtilsError)?;    // Check that the value is valid.
            is_valid_row(row, self.rows)?;                              // Check if the row exists.

            self.elements[row as usize] = value;    // Set the value to the specified vector element.

            Ok(())  // Return no error.
        }
    }

    /// This method is used to access a specific element of a vector of size m x 1.
    pub fn get_element(self: &Self, row: u8) -> Result<f32, LinalgError> {
        // Check that the vector is initialized.
        if !self.initialized {
            // The vector is not initialized.
            Err(LinalgError::NotInit)   // Return an error.
        } else {
            is_valid_row(row, self.rows)?;  // Check if the row exists.

            let value = self.elements[row as usize];    // Retrieve the value of the specified element from the vector.

            Ok(value)   // Return the value with no error.
        }
    }

    #[cfg(feature = "std")]
    /// This method is used to diplay a vector of size m x 1.
    pub fn print(self: &Self) -> Result<(), LinalgError> {
        // Iterate through each element and print it and move to the next row with a newline character.
        for row in 0..self.rows {
            let element = self.get_element(row)?;   // Retrieve the value of the specified element from the vector.
            print!("{:.3}\n", element);             // Print the value of the element.
        }

        print!("\n");   // Print an additional newline for better formatting.

        Ok(())  // Return no error.
    }

    /// This method is used to check if two vectors have the same number of rows and columns.
    pub fn is_same_size_as(self: &Self, other: &Self) -> Result<bool, LinalgError> {
        // Check that the vectors are initialized.
        if !self.initialized || !other.initialized {
            // The vector is not initialized.
            Err(LinalgError::NotInit)   // Return an error.
        } else if self.rows != other.rows {
            Ok(false)   // Return the result with no error.
        } else {
            Ok(true)    // Return the result with no error.
        }
    }

    /// This method is used to fill an entire vector of size m x 1 with a given value.
    pub fn fill(self: &mut Self, value: f32) -> Result<(), LinalgError> {
        // Assign the value to each element of the vector.
        for row in 0..self.rows {
            self.set_element(row, value)?;  // Set the value.
        }

        Ok(())  // Return no error.
    }

    /// This method is used to copy a vector of size m x 1.
    pub fn copy_from(self: &mut Self, other: &Self) -> Result<(), LinalgError> {
        // Check that the vectors have the same dimensions.
        if !self.is_same_size_as(other)? {
            // The vector do not have the same number of lines.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        for row in 0..self.rows {
            let element = other.get_element(row)?;
            self.set_element(row, element)?;
        }

        Ok(())  // Return no error.
    }

    /// This method is used to duplicate a vector of size m x 1.
    pub fn duplicate(self: &Self) -> Result<Self, LinalgError> {
        let mut duplicated_vect = Self::new();  // Create a new vector.
        duplicated_vect.init(self.rows)?;       // Initialise it with the same dimension as the original vector.

        duplicated_vect.copy_from(&self)?;      // Copy the elements of the original vector.

        Ok(duplicated_vect) // Return the duplicated vector with no error.
    }

    // This method is used to perform the vector addition operation of two vectors of size m x 1.
    pub fn add(self: &mut Self, vector1: &Self, vector2: &Self) -> Result<(), LinalgError> {
        // Check that the vectors have the same dimensions.
        if !self.is_same_size_as(vector1)? {
            // The vectors do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        // Check that the vectors have the same dimensions.
        if !self.is_same_size_as(vector2)? {
            // The vectors do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        for row in 0..self.rows {
            let element1 = vector1.get_element(row)?;
            let element2 = vector2.get_element(row)?;

            let sum = element1 + element2;

            self.set_element(row, sum)?;
        }

        Ok(())  // Return no error.
    }

    // This method is used to add another vector to itself.
    pub fn add_in_place(self: &mut Self, other: &Self) -> Result<(), LinalgError> {
        // Check that the vectors have the same dimensions.
        if !self.is_same_size_as(other)? {
            // The vectors do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        for row in 0..self.rows {
            let element1 = self.get_element(row)?;
            let element2 = other.get_element(row)?;

            let sum = element1 + element2;

            self.set_element(row, sum)?;
        }

        Ok(())  // Return no error.
    }

    // This method is used to ...
    pub fn add_scalar(self: &mut Self, scalar: f32) -> Result<(), LinalgError> {
        for row in 0..self.rows {
            let element = self.get_element(row)? + scalar;

            self.set_element(row, element)?;
        }

        Ok(())  // Return no error.
    }

    // This method is used to ...
    pub fn sub_scalar(self: &mut Self, scalar: f32) -> Result<(), LinalgError> {
        for row in 0..self.rows {
            let element = self.get_element(row)? - scalar;

            self.set_element(row, element)?;
        }

        Ok(())  // Return no error.
    }

    // This method is used to perform the vector subtraction operation of two vectors of size m x 1.
    pub fn sub(self: &mut Self, vector1: &Self, vector2: &Self) -> Result<(), LinalgError> {
        // Check that the vectors have the same dimensions.
        if !self.is_same_size_as(vector1)? {
            // The vectors do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        // Check that the vectors have the same dimensions.
        if !self.is_same_size_as(vector2)? {
            // The vectors do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        for row in 0..self.rows {
            let element1 = vector1.get_element(row)?;
            let element2 = vector2.get_element(row)?;

            let difference = element1 - element2;

            self.set_element(row, difference)?;
        }

        Ok(())  // Return no error.
    }

    // This method is used to subtract another vector to itself.
    pub fn sub_in_place(self: &mut Self, other: &Self) -> Result<(), LinalgError> {
        // Check that the vectors have the same dimensions.
        if !self.is_same_size_as(other)? {
            // The vectors do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        for row in 0..self.rows {
            let element1 = self.get_element(row)?;
            let element2 = other.get_element(row)?;

            let difference = element1 - element2;

            self.set_element(row, difference)?;
        }

        Ok(())  // Return no error.
    }

    // This method is used to calculate the norm of a vector of size m x 1.
    pub fn calculate_norm(self: &Self) -> Result<f32, LinalgError> {
        let mut norm: f32 = 0.0;

        // Iterate through each element, square it, and accumulate the sum.
        for row in 0..self.rows {
            norm += self.get_element(row)? * self.get_element(row)?;
        }

        norm = sqrtf(norm);  // Calculate the square root of the sum to obtain the Euclidean norm.

        // Check for numerical precision issues and set very small norms to zero.
        if norm <= EPSILON {
            norm = 0.0;
        }

        Ok(norm)    // Return the calculated Euclidean norm with no error.
    }

    // This method is used to normalize a vector of size m x 1.
    pub fn normalize(self: &mut Self) ->  Result<(), LinalgError> {
        let norm: f32 = self.calculate_norm()?; // Calculate the norm of the vector.

        // Normalize the vector.
        for row in 0..self.rows {
            if norm > EPSILON {
                let element: f32 = self.get_element(row)? / norm;
                self.set_element(row, element)?;
            } else {
                self.set_element(row, 0.0)?;
            }
        }

        Ok(())  // Return no error.
    }

    // This method is used to multiply by a scalar all elements of a vector of size m x 1.
    pub fn mul_by_scalar(self: &mut Self, scalar: f32) -> Result<(), LinalgError> {
        // Iterate through each element and multiply it by the scalar.
        for row in 0..self.rows {
            let element = self.get_element(row)? * scalar;
            self.set_element(row, element)?;
        }

        Ok(())  // Return no error.
    }

    // This method is used to perform dot product between two vectors of size m x 1.
    pub fn dot_product(self: &mut Self, other: &Self) -> Result<f32, LinalgError> {
        // Check that the vectors have the same dimensions.
        if !self.is_same_size_as(other)? {
            // The vectors do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        let mut scalar: f32 = 0.0;  // Initialize the result of the dot product to zero.

        // Iterate through each element, multiply corresponding elements, and accumulate the sum.
        for row in 0..self.rows {
            scalar += self.get_element(row)? * other.get_element(row)?;
        }

        Ok(scalar)  // Return the result with no error.
    }

    // This method is used to perform the spherical interpolation (SLERP) of two vectors.
    pub fn slerp(self: &mut Self, vect1: &Self, vect2: &Self, alpha: f32) -> Result<(), LinalgError> {
        // Check that alpha is valid (alpha must be between 0 and 1 inclusive).
        if !in_range(alpha, 0.0_f32, 1.0_f32) {
            // Alpha is not valid.
            return Err(LinalgError::InvalidAlpha)   // Return an error.
        }

        // Check that the vectors have the same dimensions.
        if !self.is_same_size_as(vect1)? || !self.is_same_size_as(vect2)? {
            // The vectors do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        // Perform scalar product.
        let scalar = dot_product(&vect1, &vect2)?;
        let theta = acosf(scalar);  // Angle between the axis of the first vector and the second one.

        // Create a copy of the first vector.
        let mut v1 = Self::new();
        v1.init(self.get_rows()?)?;
        v1.copy_from(&vect1)?;

        // Create a copy of the second vector.
        let mut v2 = Self::new();
        v2.init(self.get_rows()?)?;
        v2.copy_from(&vect2)?;

        v1.mul_by_scalar(sinf((1.0_f32 - alpha)*theta) / sinf(theta))?;
        v2.mul_by_scalar(sinf(alpha*theta) / sinf(theta))?;

        self.add(&v1, &v2)?;

        Ok(())    // Return no error.
    }

    // This method is used to perform the linear interpolation (LERP) of two vectors.
    pub fn lerp(self: &mut Self, vect1: &Self, vect2: &Self, alpha: f32) -> Result<(), LinalgError> {
        // Check that alpha is valid (alpha must be between 0 and 1 inclusive).
        if !in_range(alpha, 0.0_f32, 1.0_f32) {
            // Alpha is not valid.
            return Err(LinalgError::InvalidAlpha)   // Return an error.
        }

        // Check that the vectors have the same dimensions.
        if !self.is_same_size_as(vect1)? || !self.is_same_size_as(vect2)? {
            // The vectors do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        // Create a copy of the first vector.
        let mut v1 = Self::new();
        v1.init(self.get_rows()?)?;
        v1.copy_from(&vect1)?;

        // Create a copy of the second vector.
        let mut v2 = Self::new();
        v2.init(self.get_rows()?)?;
        v2.copy_from(&vect2)?;

        v1.mul_by_scalar(1.0_f32 - alpha)?;
        v2.mul_by_scalar(alpha)?;

        self.add(&v1, &v2)?;

        Ok(())    // Return no error.
    }

    /// This method is used to apply an exponent to all elements of a vector of size m x 1.
    pub fn power_elements(self: &mut Self, exponent: f32) -> Result<(), LinalgError> {
        // Iterate through each element and apply exponent.
        for row in 0..self.rows {
            let element = powf(self.get_element(row)?, exponent);
            self.set_element(row, element)?;
        }

        Ok(())  // Return no error.
    }

    /// This function is used to convert a vector of size m x 1 into a matrix of size m x 1 in order to perform matrix operations.
    pub fn convert_to_matrix(self: &Self) -> Result<Matrix, LinalgError> {
        let rows = self.get_rows()?;

        let mut mat = Matrix::new();
        mat.init(rows, 1)?;

        mat.set_col(&self, 0)?;

        Ok(mat) // Return the matrix with no error.
    }
}

// This function is used to perform dot product between two vectors of size m x 1.
pub fn dot_product(vect1: &Vector<f32>, vect2: &Vector<f32>) -> Result<f32, LinalgError> {
    // Check that the vectors have the same dimensions.
    if !vect1.is_same_size_as(vect2)? {
        // The vectors do not have the same dimensions.
        return Err(LinalgError::NotSameSize)    // Return an error.
    }

    let mut scalar: f32 = 0.0;  // Initialize the result of the dot product to zero.

    // Iterate through each element, multiply corresponding elements, and accumulate the sum.
    for row in 0..vect1.get_rows()? {
        scalar += vect1.get_element(row)? * vect2.get_element(row)?;
    }

    Ok(scalar)  // Return the result with no error.
}

// This function is used to perform the spherical interpolation (SLERP) of two vectors.
pub fn slerp(vect1: &Vector<f32>, vect2: &Vector<f32>, alpha: f32) -> Result<Vector<f32>, LinalgError> {
    let mut interpolated_vect: Vector<f32> = Vector::new();
    interpolated_vect.init(vect1.get_rows()?)?;
    interpolated_vect.slerp(&vect1, &vect2, alpha)?;

    Ok(interpolated_vect)   // Return the interpolated vector with no error.
}

// This function is used to perform the linear interpolation (LERP) of two vectors.
pub fn lerp(vect1: &Vector<f32>, vect2: &Vector<f32>, alpha: f32) -> Result<Vector<f32>, LinalgError> {
    let mut interpolated_vect: Vector<f32> = Vector::new();
    interpolated_vect.init(vect1.get_rows()?)?;
    interpolated_vect.lerp(&vect1, &vect2, alpha)?;

    Ok(interpolated_vect)   // Return the interpolated vector with no error.
}
