extern crate utils;
extern crate libm;

use utils::utils::{is_valid_value, allclose, min};
use libm::{powf, sqrtf, fabsf, acosf, sinf};

use crate::common::{M_MAX, N_MAX, EPSILON, LinalgError, is_valid_rows_number, is_valid_cols_number, is_valid_row, is_valid_col};
use crate::linalg::{lup, solve};
use crate::vector::Vector;

#[derive(Debug)]
/// Matrix structure.
pub struct Matrix {
    rows: u8,                       // Number of rows in the matrix.
    cols: u8,                       // Number of columns in the matrix.
    elements: [f32; M_MAX * N_MAX], // All the elements of the 2D matrix in a linear array.
    initialized: bool,              // This variable is used to identify whether or not a matrix has already been initialized. An uninitialized matrix cannot be manipulated.
}

impl Default for Matrix {
    fn default() -> Self {
        Self {
            rows: 0,                        // Default number of rows.
            cols: 0,                        // Default number of columns.
            elements: [0.0; M_MAX * N_MAX], // Default elements value.
            //elements: [Default::default(); M_MAX * N_MAX],
            initialized: false,             // Default initialization status.
        }
    }
}

impl Matrix {
    /// This method is used to create a new matrix of size m x n.
    pub fn new() -> Self {
        Self::default() // Create default matrix.
    }

    /// This method is used to initialize a matrix of size m x n.
    pub fn init(self: &mut Self, rows: u8, cols: u8) -> Result<(), LinalgError> {
        // Check if the matrix has already been initialized.
        if self.initialized {
            // The matrix has already been initialized.
            return Err(LinalgError::AlreadyInit);   // Return an error.
        }

        is_valid_rows_number(rows)?;    // Check that the number of rows does not exceed M_MAX.
        is_valid_cols_number(cols)?;    // Check that the number of columns does not exceed N_MAX.

        self.rows = rows;               // Set the number of rows in the matrix.
        self.cols = cols;               // Set the number of columns in the matrix.
        self.initialized = true;        // Set the initialization flag to true.

        Ok(())  // Return no error.
    }

    /// This method is used to reinitialize a matrix of size m x n.
    pub fn reinit(self: &mut Self, rows: u8, cols: u8) -> Result<(), LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            return Err(LinalgError::NotInit);   // Return an error.
        }

        self.initialized = false;   // Set the initialization flag to false.

        self.init(rows, cols)?;     // Reinitialize the matrix.

        Ok(())  // Return no error.
    }

    /// This method is used to get the number of rows (m) of a matrix of size m x n.
    pub fn get_rows(self: &Self) -> Result<u8, LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            return Err(LinalgError::NotInit);   // Return an error.
        }

        Ok(self.rows)   // Return the value with no error.
    }

    /// This method is used to set the number of rows (m) of a matrix of size m x n.
    pub(crate) fn set_rows(self: &mut Self, rows: u8) -> Result<(), LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            return Err(LinalgError::NotInit);   // Return an error.
        }

        // Check if the number of rows is different from the current one.
        if rows == self.rows {
             // The number of rows remains unchanged.
            return Err(LinalgError::UnchangedSize); // Return an error.
        }

        is_valid_rows_number(rows)?;    // Check that the number of rows does not exceed M_MAX.

        self.rows = rows;               // Set the number of rows in the matrix.

        Ok(())  // Return no error.
    }

    /// This method is used to get the number of columns (n) of a matrix of size m x n.
    pub fn get_cols(self: &Self) -> Result<u8, LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            return Err(LinalgError::NotInit);   // Return an error.
        }

        Ok(self.cols)   // Return the value with no error.
    }

    /// This method is used to set the number of columns (n) of a matrix of size m x n.
    pub(crate) fn set_cols(self: &mut Self, cols: u8) -> Result<(), LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            return Err(LinalgError::NotInit);   // Return an error.
        }

        // Check if the number of columns is different from the current one.
        if cols == self.cols {
            // The number of rows remains unchanged.
            return Err(LinalgError::UnchangedSize); // Return an error.
        }

        is_valid_cols_number(cols)?;    // Check that the number of columns does not exceed N_MAX.

        let new_elements = {
            let mut new_elements = [0.0; M_MAX * N_MAX];

            // Copy existing elements to the new array with the updated number of columns.
            for row in 0..self.rows {
                for col in 0..self.cols {
                    new_elements[row as usize * cols as usize + col as usize] = self.elements[row as usize * self.cols as usize + col as usize];
                }
            }

            new_elements
        };

        self.cols = cols;               // Set the number of columns.
        self.elements = new_elements;   // Set the new elements.

        Ok(())  // Return no error.
    }

    /// This method is used to verify if a matrix of size m x 1 is initialized or not.
    pub fn is_initialized(self: &Self) -> bool {
        self.initialized    // Return the result.
    }

    /// This method is used to assign a value to a specific element of a matrix of size m x n.
    pub fn set_element(self: &mut Self, row: u8, col: u8, value: f32) -> Result<(), LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            return Err(LinalgError::NotInit);   // Return an error.
        }

        is_valid_value(value).map_err(LinalgError::UtilsError)?;    // Check that the value is valid.

        is_valid_row(row, self.rows)?;  // Check if the row exists.
        is_valid_col(col, self.cols)?;  // Check if the column exists.

        let index: usize = (row * self.cols + col) as usize;    // Calculate the index.

        self.elements[index] = value;   // Set the value to the specified matrix element.

        Ok(())  // Return no error.
    }

    /// This method is used to access a specific element of a matrix of size m x n.
    pub fn get_element(self: &Self, row: u8, col: u8) -> Result<f32, LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            return Err(LinalgError::NotInit);   // Return an error.
        }

        is_valid_row(row, self.rows)?;  // Check if the row exists.
        is_valid_col(col, self.cols)?;  // Check if the column exists.

        let index: usize = (row * self.cols + col) as usize;    // Calculate the index.

        let value = self.elements[index];                       // Retrieve the value of the specified element from the matrix.

        Ok(value)   // Return the value with no error.
    }

    #[cfg(feature = "std")]
    /// This method is used to diplay a matrix of size m x n (not available in no_std environment).
    pub fn print(self: &Self) -> Result<(), LinalgError> {
        // Iterate through each element and print it and move to the next row with a newline character.
        for row in 0..self.rows {
            for col in 0..self.cols {
                let element = self.get_element(row, col)?;  // Retrieve the value of the specified element from the matrix.

                print!("{:.3}\t", element);                 // Print the value of the element.
            }
            print!("\n");   // Move to the next row with a newline character.
        }

        print!("\n");   // Print an additional newline for better formatting.

        Ok(())  // Return no error.
    }

    /// This method in used to replace all elements of specific row of a matrix of size m x n with those of a vector of size n x 1.
    pub fn set_row(self: &mut Self, vect: &Vector<f32>, row: u8) -> Result<(), LinalgError> {
        // Check that the number of columns in the matrix is the same as the number of rows in the vector.
        if self.cols != vect.get_rows()? {
            // The number of columns in the matrix is not the same as the number of rows in the vector.
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        // Iterate for each column of the matrix, staying on the same row.
        for col in 0..self.cols {
            self.set_element(row, col, vect.get_element(col)?)?;    // Set the new element value.
        }

        Ok(())  // Return no error.
    }

    /// This method in used to replace all elements of specific column of a matrix of size m x n with those of a vector of size m x 1.
    pub fn set_col(self: &mut Self, vect: &Vector<f32>, col: u8) -> Result<(), LinalgError> {
        // Check that the number of rows in the matrix is the same as the number of rows in the vector.
        if self.rows != vect.get_rows()? {
            // The number of rows in the matrix is not the same as the number of rows in the vector.
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        // Iterate for each row of the matrix, staying on the same column.
        for row in 0..self.rows {
            self.set_element(row, col, vect.get_element(row)?)?;    // Set the new element value.
        }

        Ok(())  // Return no error.
    }

    /// This method is used to extract all elements of specific row of a matrix of size m x n with those of a vector of size n x 1.
    pub fn get_row(self: &Self, vect: &mut Vector<f32>, row: u8) -> Result<(), LinalgError> {
        // Check that the number of columns in the matrix is the same as the number of rows in the vector.
        if self.cols != vect.get_rows()? {
            // The number of columns in the matrix is not the same as the number of rows in the vector.
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        // Iterate for each column of the matrix, staying on the same row.
        for col in 0..self.cols {
            vect.set_element(col, self.get_element(row, col)?)?;    // Retrieve element value and store it into the vector.
        }

        Ok(())  // Return no error.
    }

    /// This method is used to extract all elements of specific column of a matrix of size m x n with those of a vector of size m x 1.
    pub fn get_col(self: &Self, vect: &mut Vector<f32>, col: u8) -> Result<(), LinalgError> {
        // Check that the number of rows in the matrix is the same as the number of rows in the vector.
        if self.rows != vect.get_rows()? {
            // The number of rows in the matrix is not the same as the number of rows in the vector.
            return Err(LinalgError::InvalidSize);   // Return an error.
        }

        // Iterate for each row of the matrix, staying on the same column.
        for row in 0..self.rows {
            vect.set_element(row, self.get_element(row, col)?)?;    // Retrieve element value and store it into the vector.
        }

        Ok(())  // Return no error.
    }

    /// This method is used to extract all elements of specific row of a matrix of size m x n and store them into a new vector of size n x 1.
    pub fn row_to_vector(self: &Self, row: u8) -> Result<Vector<f32>, LinalgError> {
        let mut vect: Vector<f32> = Vector::new();
        vect.init(self.cols)?;

        self.get_row(&mut vect, row)?;

        Ok(vect)    // Return the vector with no error.
    }

    /// This method is used to extract all elements of specific column of a matrix of size m x n and store them into a new vector of size m x 1.
    pub fn col_to_vector(self: &Self, col: u8) -> Result<Vector<f32>, LinalgError> {
        let mut vect: Vector<f32> = Vector::new();
        vect.init(self.rows)?;

        self.get_col(&mut vect, col)?;

        Ok(vect)    // Return the vector with no error.
    }

    /// This method is used to add a row to a matrix of size m x n from a vector of size n x 1. The result is a matrix of size m+1 x n.
    pub fn add_row(self: &mut Self, vect: &Vector<f32>, row: u8) -> Result<(), LinalgError> {
        let rows = self.rows + 1;   // Calculate the new matrix rows number.
        self.set_rows(rows)?;       // Update it.

        // Shift current matrix elements.
        for m in (row + 1..rows).rev() {
            for col in 0..self.cols {
                self.set_element(m, col, self.get_element(m - 1, col)?)?;
            }
        }

        self.set_row(&vect, row)?;  // Add new elements to the matrix from the vector.

        Ok(())  // Return no error.
    }

    /// This method is used to add a column to a matrix of size m x n from a vector of size m x 1. The result is a matrix of size m x n+1.
    pub fn add_col(self: &mut Self, vect: &Vector<f32>, col: u8) -> Result<(), LinalgError> {
        let cols = self.cols + 1;   // Calculate the new matrix columns number.
        self.set_cols(cols)?;       // Update it.

        // Shift current matrix elements.
        for n in (col + 1..cols).rev() {
            for row in 0..self.rows {
                self.set_element(row, n, self.get_element(row, n - 1)?)?;
            }
        }

        self.set_col(&vect, col)?;  // Add new elements to the matrix from the vector.

        Ok(())  // Return no error.
    }

    /*
    pub fn del_row(self: &mut Self, row: u8) -> Result<Vector<f32, LinalgError> {
        // Add some code here.
    }

    pub fn del_column(self: &mut Self, col: u8) -> Result<Vector<f32, LinalgError> {
        // Add some code here.
    }

    pub fn swap_rows(self: &mut Self, row1: u8, row2: u8) -> Result<(), LinalgError> {
        // Add some code here. Use temporary vector to do the swipe.
    }

    pub fn swap_cols(self: &mut Self, col1: u8, col2: u8) -> Result<(), LinalgError> {
        // Add some code here. Use temporary vector to do the swipe.
    }
    */

    /// This method is used to fill an entire matrix of size m x n with a given value.
    pub fn fill(self: &mut Self, value: f32) -> Result<(), LinalgError> {
        // Assign the value to each element of the matrix.
        for row in 0..self.rows {                   // Iterate for each row of the matrix.
            for col in 0..self.cols {               // Iterate for each column of the matrix.
                self.set_element(row, col, value)?; // Set the value.
            }
        }

        Ok(())  // Return no error.
    }

    /// This method is used to check if two matrices have the same number of rows and columns.
    pub fn is_same_size_as(self: &Self, other: &Matrix) -> Result<bool, LinalgError> {
        // Check that the matrices are initialized.
        if !self.initialized || !other.initialized {
            // The matrix is not initialized.
            return Err(LinalgError::NotInit);   // Return an error.
        }

        // Check if dimensions are the same.
        if (self.rows != other.rows) || (self.cols != other.cols) {
            Ok(false)   // Return the result with no error.
        } else {
            Ok(true)    // Return the result with no error.
        }
    }

    /// This method is used to check if a matrix is square or not.
    pub fn is_square(self: &Self) -> Result<bool, LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            return Err(LinalgError::NotInit);   // Return an error.
        }

        if self.rows != self.cols {
            // The matrix has not the same number of rows as columns, it is not square.
            Ok(false)   // Return the result with no error.
        } else {
            Ok(true)    // Return the result with no error.
        }
    }

    /// This method is used to check if two matrices are identical/equal or not.
    pub fn is_equal_to(self: &Self, other: &Self, deviation: f32) -> Result<bool, LinalgError> {
        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(other)? {
            // The matrices do not have the same dimensions.
            return Ok(false);   // Return the result with no error.
        }

        // Iterate through each element and check if they are close within the deviation.
        for row in 0..self.rows {
            for col in 0..self.cols {
                let element = self.get_element(row, col)?;
                let other_element = other.get_element(row, col)?;

                let result = allclose(element, other_element, deviation).map_err(LinalgError::UtilsError)?;

                if !result {
                    return Ok(false);   // Return the result with no error.
                }
            }
        }

        Ok(true)    // Return the value with no error.
    }

    /// This method is used to fill a square matrix of size m x m with the identity matrix.
    pub fn fill_identity(self: &mut Self) -> Result<(), LinalgError> {
        // Check that the matrix is square.
        if !self.is_square()? {
            // The matrix is not square and therefore cannot be initialized as an identity matrix.
            return Err(LinalgError::NotSquare); // Return an error.
        }

        // Assign the value to each element of the matrix.
        for row in 0..self.rows {
            for col in 0..self.cols {
                if row == col {
                    self.set_element(row, col, 1.0)?;
                } else {
                    self.set_element(row, col, 0.0)?;
                }
            }
        }

        Ok(())  // Return no error.
    }

    /// This method is used to copy a matrix of size m x n.
    pub fn copy_from(self: &mut Self, other: &Matrix) -> Result<(), LinalgError> {
        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(other)? {
            // The matrices do not have the same dimensions.
            return Err(LinalgError::NotSameSize);   // Return an error.
        }

        // Iterate for each row of the matrix.
        for row in 0..self.rows {
            // Iterate for each column of the matrix.
            for col in 0..self.cols {
                let element = other.get_element(row, col)?; // Retrieve the element from the reference matrix.
                self.set_element(row, col, element)?;       // Put it inside the target matrix.
            }
        }

        Ok(())  // Return no error.
    }

    /// This method is used to duplicate a matrix of size m x n.
    pub fn duplicate(self: &Self) -> Result<Self, LinalgError> {
        let mut duplicated_mat = Self::new();       // Create a new matrix.
        duplicated_mat.init(self.rows, self.cols)?; // Initialise it with the same dimensions as the original matrix.

        duplicated_mat.copy_from(&self)?;    // Copy the elements of the original matrix.

        Ok(duplicated_mat)  // Return the duplicated vector with no error.
    }

    /// This method is used to perform the matrix addition operation of two matrices of size m x n.
    pub fn add(&mut self, matrix1: &Self, matrix2: &Self) -> Result<(), LinalgError> {
        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(matrix1)? || !self.is_same_size_as(matrix2)?{
            // The matrices do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        // Iterate for each row of the matrix.
        for row in 0..self.rows {
            // Iterate for each column of the matrix.
            for col in 0..self.cols {
                let sum =  matrix1.get_element(row, col)? + matrix2.get_element(row, col)?; // Perform elements addition.

                self.set_element(row, col, sum)?;   // Set the new value of the element.
            }
        }

        Ok(()) // Return no error.
    }

    /// This method is used to add another matrix to itself.
    pub fn add_in_place(&mut self, other: &Self) -> Result<(), LinalgError> {
        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(other)? {
            // The matrices do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        // Iterate for each row of the matrix.
        for row in 0..self.rows {
            // Iterate for each column of the matrix.
            for col in 0..self.cols {
                let sum = self.get_element(row, col)? + other.get_element(row, col)?;   // Perform elements addition.

                self.set_element(row, col, sum)?;   // Set the new value of the element.
            }
        }

        Ok(()) // Return no error.
    }

    /// This method is used to ...
    pub fn add_new(self: &Self, other: &Self) -> Result<Self, LinalgError> {
        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(other)? {
            // The matrices do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        let mut result_mat = Self::new();       // Create a new matrix to store the result.
        result_mat.init(self.rows, self.cols)?; // Initialise it with the same dimension as the original matrix.

        result_mat.add(&self, &other)?;         // Perform addition.

        Ok(result_mat)  // Return the result matrix with no error.
    }

    /// This method is used to ...
    pub fn add_scalar(self: &mut Self, other: &Self, scalar: f32) -> Result<(), LinalgError> {
        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(other)? {
            // The matrices do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        // Iterate for each row of the matrix.
        for row in 0..self.rows {
            // Iterate for each column of the matrix.
            for col in 0..self.cols {
                let element = other.get_element(row, col)? + scalar;    // Perform addition.

                self.set_element(row, col, element)?;   // Set the new value of the element.
            }
        }

        Ok(())  // Return no error.
    }

    // This method is used to ...
    pub fn add_scalar_in_place(self: &mut Self, scalar: f32) -> Result<(), LinalgError> {
        // Iterate for each row of the matrix.
        for row in 0..self.rows {
            // Iterate for each column of the matrix.
            for col in 0..self.cols {
                let element = self.get_element(row, col)? + scalar; // Perform addition.

                self.set_element(row, col, element)?;   // Set the new value of the element.
            }
        }

        Ok(())  // Return no error.
    }

    /// This method is used to ...
    pub fn add_scalar_new(self: &Self, scalar: f32) -> Result<Self, LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            return Err(LinalgError::NotInit);   // Return an error.
        }

        let mut result_mat = Self::new();       // Create a new matrix to store the result.
        result_mat.init(self.rows, self.cols)?; // Initialise it with the same dimension as the original matrix.

        // Iterate for each row of the matrix.
        for row in 0..self.rows {
            // Iterate for each column of the matrix.
            for col in 0..self.cols {
                let element = self.get_element(row, col)? + scalar; // Perform addition.

                result_mat.set_element(row, col, element)?; // Set the new value of the element.
            }
        }

        Ok(result_mat)  // Return the result matrix with no error.
    }

    /// This method is used to perform the matrix subtraction operation of two matrices of size m x n.
    pub fn sub(self: &mut Self, matrix1: &Self, matrix2: &Self) -> Result<(), LinalgError> {
        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(matrix1)? || !self.is_same_size_as(matrix2)? {
            // The matrices do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        // Iterate for each row of the matrix.
        for row in 0..self.rows {
            // Iterate for each column of the matrix.
            for col in 0..self.cols {
                let difference = matrix1.get_element(row, col)? - matrix2.get_element(row, col)?;   // Perform elements subtraction.

                self.set_element(row, col, difference)?;     // Set the new value of the element.
            }
        }

        Ok(())  // Return no error.
    }

    /// This method is used to subtract another matrix to itself.
    pub fn sub_in_place(&mut self, other: &Self) -> Result<(), LinalgError> {
        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(other)? {
            // The matrices do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        // Iterate for each row of the matrix.
        for row in 0..self.rows {
            // Iterate for each column of the matrix.
            for col in 0..self.cols {
                let difference = self.get_element(row, col)? - other.get_element(row, col)?;    // Perform elements subtraction.

                self.set_element(row, col, difference)?;    // Set the new value of the element.
            }
        }

        Ok(()) // Return no error.
    }

    /// This method is used to ...
    pub fn sub_new(self: &Self, other: &Self) -> Result<Self, LinalgError> {
        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(other)? {
            // The matrices do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        let mut result_mat = Self::new();       // Create a new matrix to store the result.
        result_mat.init(self.rows, self.cols)?; // // Initialise it with the same dimension as the original matrix.

        result_mat.sub(&self, &other)?;         // Perform addition.

        Ok(result_mat)  // Return the result matrix with no error.
    }

    /// This method is used to ...
    pub fn sub_scalar(self: &mut Self, other: &Self, scalar: f32) -> Result<(), LinalgError> {
        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(other)? {
            // The matrices do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        // Iterate for each row of the matrix.
        for row in 0..self.rows {
            // Iterate for each column of the matrix.
            for col in 0..self.cols {
                let element = other.get_element(row, col)? - scalar;    // Perform subtraction.

                self.set_element(row, col, element)?;   // Set the new value of the element.
            }
        }

        Ok(())  // Return no error.
    }

    // This method is used to ...
    pub fn sub_scalar_in_place(self: &mut Self, scalar: f32) -> Result<(), LinalgError> {
        // Iterate for each row of the matrix.
        for row in 0..self.rows {
            // Iterate for each column of the matrix.
            for col in 0..self.cols {
                let element = self.get_element(row, col)? - scalar; // Perform subtraction.

                self.set_element(row, col, element)?;   // Set the new value of the element.
            }
        }

        Ok(())  // Return no error.
    }

    /// This method is used to ...
    pub fn sub_scalar_new(self: &Self, scalar: f32) -> Result<Self, LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            return Err(LinalgError::NotInit);   // Return an error.
        }

        let mut result_mat = Self::new();       // Create a new matrix to store the result.
        result_mat.init(self.rows, self.cols)?; // Initialise it with the same dimension as the original matrix.

        result_mat.sub_scalar(&self, scalar)?;  // Perform subtraction.

        Ok(result_mat)  // Return the result matrix with no error.
    }

    /// This method is used to perform the matrix multiplication operation on two matrices of sizes m x n and (n x k) respectively.
    // Naive implementation, as it is not very efficient for large matrices (for larger matrices, use the "divide and conquer" strategy).
    pub fn mul(self: &mut Self, matrix1: &Self, matrix2: &Self) -> Result<(), LinalgError> {
        if matrix1.cols != matrix2.rows || self.rows != matrix1.rows || self.cols != matrix2.cols {
            return Err(LinalgError::InvalidSize)    // Return an error.
        }

        for row in 0..matrix1.rows {
            for col2 in 0..matrix2.cols {
                let mut element: f32 = 0.0;

                for col1 in 0..matrix1.cols {
                    let element1 = matrix1.get_element(row, col1)?;
                    let element2 = matrix2.get_element(col1, col2)?;

                    element += element1 * element2;

                    self.set_element(row, col2, element)?;
                }
            }
        }

        Ok(())  // Return no error.
    }

    /// This method is used to ...
    pub fn mul_new(self: &Self, other: &Self) -> Result<Self, LinalgError> {
        let mut result_mat = Self::new();
        let m = self.get_rows()?;
        let k = other.get_cols()?;

        result_mat.init(m, k)?;

        result_mat.mul(&self, &other)?;

        Ok(result_mat)  // Return no error.
    }

    /// This method is used to transpose a matrix of size m x n.
    pub fn transpose(self: &mut Self) -> Result<(), LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            return Err(LinalgError::NotInit);   // Return an error.
        }

        // Check if the matrix is square.
        if self.is_square()? {
            // The matrix is square.
            for row in 0..self.rows {
                for col in (row + 1)..self.cols {
                    let temp_element = self.get_element(row, col)?;

                    self.set_element(row, col, self.get_element(col, row)?)?;
                    self.set_element(col, row, temp_element)?;
                }
            }
        } else {
            // TODO : optimize it if it's possible.
            // The matrix is rectangular.
            let mut temp: Self = Self::new();
            temp.init(self.cols, self.rows)?;

            for row in 0..self.rows {
                for col in 0..self.cols {
                    temp.set_element(col, row, self.get_element(row, col)?)?;
                }
            }

            // Change the size of the matrix.
            let rows: u8 = self.cols;
            let cols: u8 = self.rows;
            self.rows = rows;
            self.cols = cols;

            self.copy_from(&temp)?;
        }

        Ok(())
    }

    /// This method is used to ...
    pub fn transpose_new(self: &Self) -> Result<Matrix, LinalgError> {
        let mut transposed_mat = self.duplicate()?; // Duplicate the original matrix to avoid modifying it.
        transposed_mat.transpose()?;                // Transpose the copy.

        Ok(transposed_mat)  // Return the transposed matrix with no error.
    }

    /// This method is used to invert a square matrix of size m x m.
    pub fn invert(self: &mut Self) -> Result<(), LinalgError> {
        // Check that the matrix is square.
        if !self.is_square()? {
            // The matrix is not square.
            return Err(LinalgError::NotSquare); // Return an error.
        }

        let mut tmp_vec: Vector<f32> = Vector::new();
        tmp_vec.init(self.rows)?;


        // Apply PLU decomposition and check if the determinant is null.
        let (lu, p) = match lup(&self) {
            Ok(result) => result,
            Err(_) => return Err(LinalgError::NotInversible),
        };

        for row in 0..self.rows {
            tmp_vec.set_element(row, 1.0)?;

            let x = match solve(&lu, &tmp_vec, &p) {
                Ok(result) => result,
                Err(_) => return Err(LinalgError::NotInversible),
            };

            for col in 0..self.cols {
                self.set_element(row, col, x.get_element(col)?)?;
            }

            tmp_vec.set_element(row, 0.0)?;
        }

        self.transpose()?;  // Transpose the result to get the inverse.

        Ok(())
    }

    /// This method is used to multiply by a scalar all elements of a matrix of size m x n.
    pub fn mul_by_scalar_in_place(self: &mut Self, scalar: f32) -> Result<(), LinalgError> {
        // Iterate through each element and multiply it by the scalar.
        for row in 0..self.rows {
            for col in 0..self.cols {
                let element = self.get_element(row, col)? * scalar;
                self.set_element(row, col, element)?;
            }
        }

        Ok(())  // Return no error.
    }

    /// This method is used to apply an exponent to all elements of a matrix of size m x n.
    pub fn power_elements(self: &mut Self, exponent: f32) -> Result<(), LinalgError> {
        // Iterate through each element and apply exponent.
        for row in 0..self.rows {
            for col in 0..self.cols {
                let element = powf(self.get_element(row, col)?, exponent);
                self.set_element(row, col, element)?;
            }
        }

        Ok(())  // Return no error.
    }

    /*
    pub fn power(self: &mut Self, exponent: u8) -> Result<(), LinalgError> {
        for n in 0..exponent {

        }
    }
    */

    /// This method is used to calculate the trace of a matrix of size m x n.
    pub fn trace(self: &Self) -> Result<f32, LinalgError> {
        // Check that the matrix is square.
        if !self.is_square()? {
            // The matrix is not square.
            return Err(LinalgError::NotSquare); // Return an error.
        }

        let mut trace: f32 = 0.0;

        // Loop through the minimum of m and n and accumulate diagonal elements to trace.
        for i in 0..min(self.rows, self.cols) {
            trace += self.get_element(i, i)?;   // Add the element at position (i, i) to the trace.
        }

        Ok(trace)   // Return the computed trace with no error.
    }

    /*
        Notes:

        QR decomposition is a matrix decomposition designed to factor a rectangular matrix into two matrices.
        One (Q) is orthogonal (or unitary in the complex case) and the other (R) is upper triangular (or upper triangular in the complex case).
        The matrix Q is of size m x n and orthogonal (or unitary) which means that for the real case: Trans(Q).Q = I and for the complex case: Q*.Q = I.
        Where Trans(Q) is the transpose of Q and Q* is its transposed conjugate.
        The matrix R is upper triangular (or tri-superior) of size m x n which means that all elements below the main diagonal of R are zero.
    */

    /// This method is used to perform QR decomposition of a matrix of size m x n.
    pub fn qr_decomp(self: &Self) -> Result<(Self, Self), LinalgError> {
        let mut q: Matrix = Matrix::new();
        q.init(self.rows, self.cols)?;
        q.copy_from(&self)?;

        let mut r: Matrix = Matrix::new();
        r.init(self.rows, self.cols)?;

        let mut tmp: Matrix = Matrix::new();
        tmp.init(self.rows, self.cols)?;

        // Apply the Gram-Schmidt method to orthogonalise the columns and obtain the Q matrix.
        for col in 0..self.cols {
            for row in 0..col {
                let mut dot_product: f32 = 0.0;

                for k in 0..self.rows {
                    dot_product += q.get_element(k, row)? * q.get_element(k, col)?;
                }

                for k in 0..self.rows {
                    let element = q.get_element(k, col)? - dot_product * q.get_element(k, row)?;
                    q.set_element(k, col, element)?;
                }
            }

            // Normalize the column.
            let mut norm: f32 = 0.0;

            for row in 0..self.rows {
                norm += q.get_element(row, col)? * q.get_element(row, col)?;
            }

            norm = sqrtf(norm);

            if norm > EPSILON {
                for k in 0..self.rows {
                    let element = q.get_element(k, col)? / norm;
                    q.set_element(k, col, element)?;
                }
            } else {
                break;
            }
        }

        tmp.copy_from(&q)?;
        tmp.transpose()?;
        r.mul(&tmp, &self)?;

        Ok((q, r))
    }

    /*
    /// This method is used to check if a matrix of size m x n is in row echelon form.
    pub fn is_row_echelon_form(self: &Self) -> Result<bool, LinalgError> {

    }
    */

    /// This method is used to check if a matrix of size m x n is upper triangular.
    pub fn is_upper_triangular(self: &Self) -> Result<bool, LinalgError> {
        // Loops on every row except the first (which doesn't need to have null elements).
        for row in 1..self.rows {
            // We loop for the columns corresponding to the lower triangular matrix as a function of the current row.
            for col in 0..row {
                // If the matrix is upper diagonal, the element must always be zero, otherwise the matrix cannot be upper diagonal.
                if !allclose(self.get_element(row, col)?, 0.0, EPSILON).map_err(LinalgError::UtilsError)? {
                    return Ok(false);   // Return the result with no error.
                }
            }
        }

        Ok(true)    // Return the result with no error.
    }

    /// This method is used to check if a matrix of size m x n is lower triangular.
    pub fn is_lower_triangular(self: &Self) -> Result<bool, LinalgError> {
        // Loops on every row except the last (which doesn't need to have null elements).
        for row in 0..self.rows - 1 {
            // We loop for the columns corresponding to the upper triangular matrix as a function of the current row.
            for col in row + 1..self.cols {
                // If the matrix is lower triangular, the element must always be zero, otherwise the matrix cannot be lower triangular.
                if !allclose(self.get_element(row, col)?, 0.0, EPSILON).map_err(LinalgError::UtilsError)? {
                    return Ok(false);   // Return the result with no error.
                }
            }
        }

        Ok(true)    // Return the result with no error.
    }

    /// This method is used to calculate the frobenius norm of a matrix of size m x n.
    pub fn calculate_frobenius_norm(self: &Self) -> Result<f32, LinalgError> {
        let mut frobenius_norm: f32 = 0.0;

        // Iterate through each element, square it, and accumulate the sum.
        for row in 0..self.rows {
            for col in 0..self.cols {
                frobenius_norm += self.get_element(row, col)? * self.get_element(row, col)?;
            }
        }

        frobenius_norm = sqrtf(frobenius_norm);

        Ok(frobenius_norm)
    }

    /// This method is used to calculate the derminant of a matrix of size m x m.
    pub fn det(self: &Self) -> Result<f32, LinalgError> {
        let mut determinant: f32 = 1.0;

        // Check that the matrix is square.
        if !self.is_square()? {
            // The matrix is not square.
            return Err(LinalgError::NotSquare); // Return an error.
        }

        let (lu, p) = lup(&self)?;  // Apply PLU decomposition and check if the determinant is null.

        for row in 0..self.rows {
            determinant *= lu.get_element(p.get_element(row)?, row)?;
        }

        let mut col: u8 = 0;

        for row in 0..self.rows {
            if p.get_element(row)? != row {
                col += 1;
            }
        }

        if col != 0 && (col - 1) % 2 == 1 {
            determinant = -determinant;
        }

        Ok(determinant) // Return the determinant with no error.
    }

    /// This method is used to check if a matrix of size m x n is symmetrical or not.
    pub fn is_symmetric(self: &Self) -> Result<bool, LinalgError> {
        // Check that the matrix is square.
        if !self.is_square()? {
            // A non-square matrix can't be symmetric.
            return Ok(false);   // Return the result with no error.
        }

        for row in 0..self.rows {
            for col in 0..self.cols {
                // If any element is not equal to its symmetric counterpart, the matrix is not symmetric.
                if !allclose(self.get_element(row, col)?, self.get_element(col, row)?, EPSILON).map_err(LinalgError::UtilsError)? {
                    return Ok(false);   // Return the result with no error.
                }
            }
        }

        Ok(true)    // Return the result with no error.
    }

    /// This method is used to ...
    pub fn is_positive_definite(self: &Self) -> Result<bool, LinalgError> {
        // Check that the matrix is symetric.
        if !self.is_symmetric()? {
            // The matrix is not symmetric, and so has no Cholesky decomposition.
            return Err(LinalgError::NotSymetric);   // Return an error.
        }

        for k in 0..self.rows {
            // Create the submatrix.
            let mut tmp = Matrix::new();
            tmp.init(k + 1, k + 1)?;

            // Copy the elements to submatrix.
            for row in 0..=k {
                for col in 0..=k {
                    tmp.set_element(row, col, self.get_element(row, col)?)?;
                }
            }

            // Check the determinant of the submatrix.
            if tmp.det()? < 0.0 {
                return Ok(false);
            }
        }

        Ok(true)    // Return the result with no error.
    }

    /// This method is used to perform Cholesky decomposition of a Hermitian positive-definite matrix (A=LL^T).
    pub fn chol(self: &Self) -> Result<Self, LinalgError> {
        // Check that the matrix is positive-definite.
        if !self.is_positive_definite()? {
            // The matrix is not positive-definite, and so has no Cholesky decomposition.
            return Err(LinalgError::NotPositiveDefinite);   // Return an error.
        }

        let mut l = Matrix::new();
        l.init(self.rows, self.cols)?;

        /*
        // Method 1.
        for row in 0..self.rows {
            for col in 0..=row {
                let mut s: f32 = 0.0;

                for k in 0..col {
                    s *= l.get_element(row, k)? * l.get_element(col, k)?;
                }

                // We cannot divide by zero.
                if l.get_element(col, col)? < EPSILON {
                    l.set_element(col, col, EPSILON)?;
                }

                l.set_element(
                    row,
                    col,
                    if row == col {
                        sqrtf(self.get_element(row, row)? - s)
                    } else {
                        1.0 / l.get_element(col, col)? * (self.get_element(row, col)? - s)
                    }
                )?;
            }
        }
        */

        // Method 2.
        for col in 0..self.cols {
            let mut s1: f32 = 0.0;

            for k in 0..col {
                s1 += l.get_element(col, k)? * l.get_element(col, k)?;
            }

            // We cannot divide by zero.
            if l.get_element(col, col)? < EPSILON {
                l.set_element(col, col, EPSILON)?;
            }

            for row in 0..self.rows {
                let mut s2: f32 = 0.0;

                for k in 0..col {
                    s2 += l.get_element(row, k)? * l.get_element(col, k)?;
                }

                l.set_element(
                    row,
                    col,
                    if row == col {
                        sqrtf(self.get_element(row, col)? - s1)
                    } else {
                        1.0 / l.get_element(col, col)? * (self.get_element(row, col)? - s2)
                    },
                )?;
            }
        }

        Ok(l)
    }

    /*
    pub fn cholupdate(self: &Self) -> Result<Self, LinalgError> {
        // Check that the matrix is square.
        if !self.is_square()? {
            // A non-square matrix can't be symmetric.
            return Ok(false);   // Return the result with no error.
        }
    }
    */

    /// This method is used to ...
    pub fn dlyap(self: &Self, q: &Self) -> Result<Self, LinalgError> {
        // Check that the matrix is square.
        if !self.is_square()? {
            // The matrix is not square.
            return Err(LinalgError::NotSquare); // Return an error.
        }

        // Check that the matrix is positive-definite.
        if !q.is_positive_definite()? {
            // The matrix is not positive-definite, and so has no Cholesky decomposition.
            return Err(LinalgError::NotPositiveDefinite);   // Return an error.
        }

        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(q)? {
            // The matrices do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        let mut p: Matrix = Matrix::new();
        p.init(self.rows, self.cols)?;

        Ok(p)
    }

    /// This method is used to check if a matrix of size m x n is orthogonal or not.
    pub fn is_orthogonal(self: &Self) -> Result<bool, LinalgError> {
        let m = self.rows;

        let determinant_abs = fabsf(self.det()?);   // Calculate the absolute value of the determinant of the matrix.

        // Check if the determinant is equal to -1 or 1.
        if !allclose(determinant_abs, 1.0_f32, EPSILON).map_err(LinalgError::UtilsError)? {
            // The absolute value of the determinant is different from 1 so can't be orthogonal.
            return Ok(false);   // Return false wwith no error.
        }

        // Create a copy of the actual matrix and transpose it.
        let mut transposed_mat = self.duplicate()?;
        transposed_mat.transpose()?;

        // Create the identity matrix.
        let mut identity_mat = Self::new();
        identity_mat.init(m, m)?;
        identity_mat.fill_identity()?;

        let result_mat = self.mul_new(&transposed_mat)?;

        // Check that the relationship R Tr(R) = I is satisfied.
        if !result_mat.is_equal_to(&identity_mat, EPSILON)? {
            // The product of the matrix and its transpose is not equal to the identity matrix and so cannot be orthogonal.
            return Ok(false);   // Return false wwith no error.
        }

        Ok(true)    // Return true with no error.
    }

    /// This method can be used to check if a matrix of size m x n belongs to the SO(n) group or not.
    pub fn is_so_n(self: &Self) -> Result<bool, LinalgError> {
        let m = self.rows;

        // Check if the determinant is equal to 1.
        let determinant = self.det()?;

        if !allclose(determinant, 1.0_f32, EPSILON).map_err(LinalgError::UtilsError)? {
            // The absolute value of the determinant is different from 1 so can't be orthogonal.
            return Ok(false);   // Return false wwith no error.
        }

        // Create a copy of the actual matrix and transpose it.
        let mut transposed_mat = self.duplicate()?;
        transposed_mat.transpose()?;

        // Create the identity matrix.
        let mut identity_mat = Self::new();
        identity_mat.init(m, m)?;
        identity_mat.fill_identity()?;

        let result_mat = self.mul_new(&transposed_mat)?;

        // Check that the relationship R Tr(R) = I is satisfied.
        if !result_mat.is_equal_to(&identity_mat, EPSILON)? {
            // The product of the matrix and its transpose is not equal to the identity matrix and so cannot be orthogonal.
            return Ok(false);   // Return false wwith no error.
        }

        Ok(true)    // Return true with no error.
    }

    /// This method is used to extract a specified diagonal of a matrix of size m x n.
    // k: int, optional, is the diagonal in question. The default is 0. Use k>0 for diagonals above the main diagonal, and k<0 for diagonals below the main diagonal.
    pub fn diag(self: &Self, k: Option<i8>) -> Result<Vector<f32>, LinalgError> {
        // TODO: check if the matrix is initialized.
        let mut vect: Vector<f32> = Vector::new();

        // Retrieve matrix dimensions.
        let m = self.rows as i8;
        let n = self.cols as i8;

        // Try to retrieve the diagonal to extract.
        let k = k.unwrap_or(0);

        // Calculate the lenght of the diagonal to extract.
        let diag_len = if k >= 0 {
            min(m, n - k)
        } else {
            min(m + k, n)
        };

        // Check if the diagonal length is valid.
        if diag_len <= 0 {
            return Err(LinalgError::InvalidDiagonalLength); // Return an error.
        }

        vect.init(diag_len as u8)?;

        for i in 0..diag_len {
            let row = if k >= 0 { i } else { i - k };
            let col = if k >= 0 { i + k } else { i };
            let element = self.get_element(row as u8, col as u8)?;
            vect.set_element(i as u8, element)?;
        }

        Ok(vect)    // Return the diagonal as a vector with no error.
    }

    /// This method is used to calculate the logarithmic map of a rotation matrix or a Direct Cosine Matrix (DCM).
    /*
    It corresponds to the logarithm given by the Rodrigues rotation formula:

    \log(\mathbf{R}) = \frac{\theta(\mathbf{R} - \mathbf{R}^T)}{2\sin\theta}
    \theta = \arccos\left(\frac{\mathrm{tr}(\mathbf{R}) - 1}{2}\right)
    */
    pub fn log(self: &Self) -> Result<Self, LinalgError> {
        /*
        \log(\mathbf{R}) = \frac{\theta(\mathbf{R} - \mathbf{R}^T)}{2\sin\theta}
        \theta = \arccos\left(\frac{\mathrm{tr}(\mathbf{R}) - 1}{2}\right)
        */

        // Check if the matrix belongs to SO(n).
        if !self.is_so_n()? {
            // The matrix does not belong to SO(n).
            //return Err(LinalgError::); // Return an error.
        }

        let theta = acosf((self.trace()? - 1.0) / 2.0);

        let mut transposed_mat = self.duplicate()?; // Duplicate the actual matrix.
        transposed_mat.transpose()?;                // Tranpose it.

        let mut result_mat = self.sub_new(&transposed_mat)?;
        result_mat.mul_by_scalar_in_place(theta / (2.0 * sinf(theta)))?;

        Ok(result_mat)  // Return the result matrix with no error.
    }

    /// This method is used to calculate the logarithmic map of a rotation matrix or a Direct Cosine Matrix (DCM).
    /*
    It corresponds to the logarithm given by the Rodrigues rotation formula:

    \log(\mathbf{R}) = \frac{\theta(\mathbf{R} - \mathbf{R}^T)}{2\sin\theta}
    \theta = \arccos\left(\frac{\mathrm{tr}(\mathbf{R}) - 1}{2}\right)
    */
    pub fn log_in_place(self: &mut Self) -> Result<(), LinalgError> {
        // Check if the matrix belongs to SO(n).
        if !self.is_so_n()? {
            // The matrix does not belong to SO(n).
            //return Err(LinalgError::); // Return an error.
        }

        let theta = acosf((self.trace()? - 1.0) / 2.0);

        let mut transposed_mat = self.duplicate()?; // Duplicate the actual matrix.
        transposed_mat.transpose()?;                // Tranpose it.

        self.sub_in_place(&transposed_mat)?;
        self.mul_by_scalar_in_place(theta / (2.0 * sinf(theta)))?;

        Ok(())  // Return no error.
    }

    /*
    pub fn adjugate(self: &Self) -> Result<Self, LinalgError> {
        //
    }
    */

    /// This method is used to ...
    pub fn adjugate_in_place(self: &mut Self) -> Result<(), LinalgError> {
        // Check if the matrix belongs to SO(n).
        if !self.is_so_n()? {
            // The matrix does not belong to SO(n).
            //return Err(LinalgError::); // Return an error.
        }

        let determinant = self.det()?;  // Calculate the determinant of the matrix.

        self.transpose()?;              // Tranpose the matrix.

        self.mul_by_scalar_in_place(determinant)?;

        Ok(())  // Return no error.
    }
}
