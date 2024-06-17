extern crate utils;
extern crate libm;

use utils::utils::{is_valid_value, close_all, min};
use libm::{powf, sqrtf, fabsf};
use crate::common::{M_MAX, N_MAX, EPSILON, LinalgError};
use crate::common::{is_valid_rows_number, is_valid_cols_number, is_valid_row, is_valid_col};
use crate::linalg::{lup, solve};
use crate::vector::Vector;

#[derive(Debug)]
pub struct Matrix {
    rows: u8,                       // Number of rows in the matrix.
    cols: u8,                       // Number of columns in the matrix.
    elements: [f32; M_MAX * N_MAX], // All the elements of the 2D matrix in a linear array.
    initialized: bool,              // This variable is used to identify whether or not a matrix has already been initialized. An uninitialized matrix cannot be manipulated.
}

impl Matrix {
    // This function is used to create a new matrix of size m x n.
    pub fn new() -> Self {
        Self {
            rows: 0,
            cols: 0,
            elements: [0.0; M_MAX * N_MAX],
            initialized: false,
        }
    }

    // This function is used to initialize a matrix of size m x n.
    pub fn init(self: &mut Self, rows: u8, cols: u8) -> Result<(), LinalgError> {
        // Check if the matrix has already been initialized.
        if self.initialized {
            // The matrix has already been initialized.
            Err(LinalgError::AlreadyInit)   // Return an error.
        } else {
            is_valid_rows_number(rows)?;    // Check that the number of rows does not exceed M_MAX.
            is_valid_cols_number(cols)?;    // Check that the number of columns does not exceed N_MAX.

            self.rows = rows;           // Set the number of rows in the matrix.
            self.cols = cols;           // Set the number of columns in the matrix.
            self.initialized = true;    // Set the initialization flag to true.

            Ok(())  // Return no error.
        }
    }

    // This function is used to reinitialize a matrix of size m x n.
    pub fn reinit(self: &mut Self, rows: u8, cols: u8) -> Result<(), LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            Err(LinalgError::NotInit)   // Return an error.
        } else {
            self.initialized = false;   // Set the initialization flag to false.

            self.init(rows, cols)?;     // Reinitialize the matrix.

            Ok(())  // Return no error.
        }
    }

    // This function is used to get the number of rows (m) of the matrix of size m x n.
    pub fn get_rows(self: &Self) -> Result<u8, LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            Err(LinalgError::NotInit)   // Return an error.
        } else {
            Ok(self.rows)   // Return the value with no error.
        }
    }

    // This function is used to set the number of rows (m) of the matrix of size m x n.
    pub(crate) fn set_rows(self: &mut Self, rows: u8) -> Result<(), LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            Err(LinalgError::NotInit)   // Return an error.
        } else if rows == self.rows {
            Err(LinalgError::UnchangedSize) // Return an error.
        } else {
            is_valid_rows_number(rows)?;    // Check that the number of rows does not exceed M_MAX.
            self.rows = rows;               // Set the number of rows in the matrix.

            Ok(())  // Return no error.
        }
    }

    // This function is used to get the number of columns (n) of the matrix of size m x n.
    pub fn get_cols(self: &Self) -> Result<u8, LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            Err(LinalgError::NotInit)   // Return an error.
        } else {
            Ok(self.cols)   // Return the value with no error.
        }
    }

    // This function is used to set the number of columns (n) of the matrix of size m x n.
    pub(crate) fn set_cols(self: &mut Self, cols: u8) -> Result<(), LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            Err(LinalgError::NotInit)   // Return an error.
        } else if cols == self.cols {
            Err(LinalgError::UnchangedSize) // Return an error.
        } else {
            is_valid_cols_number(cols)?;    // Check that the number of columns does not exceed N_MAX.
            //self.cols = cols;               // Set the number of columns in the matrix.

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
    }

    // This function is used to verify if the matrix is initialized or not.
    pub fn is_initialized(self: &Self) -> bool {
        self.initialized
    }

    // This function is used to assign a value to a specific element of a matrix of size m x n.
    pub fn set_element(self: &mut Self, row: u8, col: u8, value: f32) -> Result<(), LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            Err(LinalgError::NotInit)   // Return an error.
        } else {
            is_valid_value(value).map_err(LinalgError::UtilsError)?;    // Check that the value is valid.

            is_valid_row(row, self.rows)?;  // Check if the row exists.
            is_valid_col(col, self.cols)?;  // Check if the column exists.

            let index: usize = (row * self.cols + col) as usize;    // Calculate the index.
            self.elements[index] = value;                           // Set the value to the specified matrix element.

            Ok(())  // Return no error.
        }
    }

    // This function is used to access a specific element of a matrix of size m x n.
    pub fn get_element(self: &Self, row: u8, col: u8) -> Result<f32, LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            Err(LinalgError::NotInit)   // Return an error.
        } else {
            is_valid_row(row, self.rows)?;  // Check if the row exists.
            is_valid_col(col, self.cols)?;  // Check if the column exists.

            let index: usize = (row * self.cols + col) as usize;    // Calculate the index.
            let value = self.elements[index];                       // Retrieve the value of the specified element from the matrix.

            //is_valid_value(value).map_err(LinalgError::UtilsError)?;

            Ok(value)   // Return the value with no error.
        }
    }

    // This function is used to fill an entire matrix of size m x n with a given value.
    pub fn fill(self: &mut Self, value: f32) -> Result<(), LinalgError> {
        // Assign the value to each element of the matrix.
        for row in 0..self.rows {
            for col in 0..self.cols {
                self.set_element(row, col, value)?; // Set the value.
            }
        }

        Ok(())  // Return no error.
    }

    // This function is used to check if a matrix is square or not.
    pub fn is_square(self: &Self) -> Result<bool, LinalgError> {
        // Check that the matrix is initialized.
        if !self.initialized {
            // The matrix is not initialized.
            Err(LinalgError::NotInit)   // Return an error.
        } else if self.rows != self.cols {
            // The matrix has not the same number of rows as columns, it is not square.
            Ok(false)   // Return the result with no error.
        } else {
            Ok(true)    // Return the result with no error.
        }
    }

    // This function is used to fill a square matrix of size (m x m) with the identity matrix.
    pub fn fill_identity(self: &mut Self) -> Result<(), LinalgError> {
        // Check that the matrix is square.
        if !self.is_square()? {
            // The matrix is not square.
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

    // This function is used to check if two matrices have the same number of rows and columns.
    pub fn is_same_size_as(self: &Self, other: &Matrix) -> Result<bool, LinalgError> {
        // Check that the matrices are initialized.
        if !self.initialized || !other.initialized {
            // The matrix is not initialized.
            Err(LinalgError::NotInit)   // Return an error.
        } else if (self.rows != other.rows) || (self.cols != other.cols) {
            Ok(false)   // Return the result with no error.
        } else {
            Ok(true)    // Return the result with no error.
        }
    }

    // This function is used to duplicate/copy a matrix of size m x n.
    pub fn copy_from(self: &mut Self, other: &Matrix) -> Result<(), LinalgError> {
        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(other)? {
            // The matrices do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        for row in 0..self.rows {
            for col in 0..self.cols {
                let element = other.get_element(row, col)?; // Retrieve the element from the reference matrix.
                self.set_element(row, col, element)?;       // Put it inside the target matrix.
            }
        }

        Ok(())  // Return no error.
    }

    // This function is used to check if two matrices are identical/equal or not.
    pub fn is_equal_to(self: &Self, other: &Matrix, deviation: f32) -> Result<bool, LinalgError> {
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

                let result = close_all(element, other_element, deviation).map_err(LinalgError::UtilsError)?;

                if !result {
                    return Ok(false);   // Return the result with no error.
                }
            }
        }

        Ok(true)    // Return the value with no error.
    }

    #[cfg(feature = "std")]
    // This function is used to diplay a matrix of size m x n.
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

    // This function is used to perform the matrix addition operation of two matrices of size m x n.
    // A = B + C.
    pub fn add(&mut self, matrix1: &Self, matrix2: &Self) -> Result<(), LinalgError> {
        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(matrix1)? {
            // The matrices do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(matrix2)? {
            // The matrices do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        for row in 0..self.rows {
            for col in 0..self.cols {
                let element1 = matrix1.get_element(row, col)?;
                let element2 = matrix2.get_element(row, col)?;

                let sum = element1 + element2;

                self.set_element(row, col, sum)?;
            }
        }

        Ok(()) // Return no error.
    }

    // This function is used to add another matrix to itself.
    pub fn add_in_place(&mut self, other: &Self) -> Result<(), LinalgError> {
        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(other)? {
            // The matrices do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        for row in 0..self.rows {
            for col in 0..self.cols {
                let element1 = self.get_element(row, col)?;
                let element2 = other.get_element(row, col)?;

                let sum = element1 + element2;

                self.set_element(row, col, sum)?;
            }
        }

        Ok(()) // Return no error.
    }

    // This function is used to perform the matrix subtraction operation of two matrices of size m x n.
    pub fn sub(self: &mut Self, matrix1: &Self, matrix2: &Self) -> Result<(), LinalgError> {
        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(matrix1)? {
            // The matrices do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(matrix2)? {
            // The matrices do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        for row in 0..self.rows {
            for col in 0..self.cols {
                let element1 = matrix1.get_element(row, col)?;
                let element2 = matrix2.get_element(row, col)?;

                let difference = element1 - element2;

                self.set_element(row, col, difference)?;
            }
        }

        Ok(())  // Return no error.
    }

    // This function is used to subtract another matrix to itself.
    pub fn sub_in_place(&mut self, other: &Self) -> Result<(), LinalgError> {
        // Check that the matrices have the same dimensions.
        if !self.is_same_size_as(other)? {
            // The matrices do not have the same dimensions.
            return Err(LinalgError::NotSameSize)    // Return an error.
        }

        for row in 0..self.rows {
            for col in 0..self.cols {
                let element1 = self.get_element(row, col)?;
                let element2 = other.get_element(row, col)?;

                let difference = element1 - element2;

                self.set_element(row, col, difference)?;
            }
        }

        Ok(()) // Return no error.
    }

    // This function is used to perform the matrix multiplication operation on two matrices of sizes m x n and (n x k) respectively.
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

    // This function is used to transpose a matrix of size m x n.
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
            // TO DO : optimize if if it's possible.
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

    // This function is used to invert a square matrix of size (m x m)
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

    // This function is used to multiply by a scalar all elements of a matrix of size m x n.
    pub fn mul_by_scalar(self: &mut Self, scalar: f32) -> Result<(), LinalgError> {
        // Iterate through each element and multiply it by the scalar.
        for row in 0..self.rows {
            for col in 0..self.cols {
                let element = self.get_element(row, col)? * scalar;
                self.set_element(row, col, element)?;
            }
        }

        Ok(())  // Return no error.
    }

    // This function is used to apply an exponent to all elements of a matrix of size m x n.
    pub fn power_exponent(self: &mut Self, exponent: f32) -> Result<(), LinalgError> {
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

    // This function is used to calculate the trace of a matrix of size m x n.
    pub fn trace(self: &Self) -> Result<f32, LinalgError> {
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
        The matrix Q is of size (m x m) and orthogonal (or unitary) which means that for the real case: Trans(Q).Q = I and for the complex case: Q*.Q = I.
        Where Trans(Q) is the transpose of Q and Q* is its transposed conjugate.
        The matrix R is upper triangular (or tri-superior) of size m x n which means that all elements below the main diagonal of R are zero.
    */

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
    pub fn is_row_echelon_form(self: &Self) -> Result<bool, LinalgError> {

    }
    */

    pub fn is_upper_triangular(self: &Self) -> Result<bool, LinalgError> {
        // Loops on every row except the first (which doesn't need to have null elements).
        for row in 1..self.rows {
            // We loop over the columns corresponding to the lower triangular matrix as a function of the current row.
            for col in 0..row {
                // If the matrix is upper diagonal, the element must always be zero, otherwise the matrix cannot be upper diagonal.
                if !close_all(self.get_element(row, col)?, 0.0, EPSILON).map_err(LinalgError::UtilsError)? {
                    return Ok(false);   // Return the result with no error.
                }
            }
        }

        Ok(true)    // Return the result with no error.
    }

    pub fn is_lower_triangular(self: &Self) -> Result<bool, LinalgError> {
        // Loops on every row except the last (which doesn't need to have null elements).
        for row in 0..self.rows - 1 {
            // We loop over the columns corresponding to the upper triangular matrix as a function of the current row.
            for col in row + 1..self.cols {
                // If the matrix is lower triangular, the element must always be zero, otherwise the matrix cannot be lower triangular.
                if !close_all(self.get_element(row, col)?, 0.0, EPSILON).map_err(LinalgError::UtilsError)? {
                    return Ok(false);   // Return the result with no error.
                }
            }
        }

        Ok(true)    // Return the result with no error.
    }

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

        Ok(determinant)
    }

    pub fn is_symmetric(self: &Self) -> Result<bool, LinalgError> {
        // Check that the matrix is square.
        if !self.is_square()? {
            // A non-square matrix can't be symmetric.
            return Ok(false);   // Return the result with no error.
        }

        for row in 0..self.rows {
            for col in 0..self.cols {
                // If any element is not equal to its symmetric counterpart, the matrix is not symmetric.
                if !close_all(self.get_element(row, col)?, self.get_element(col, row)?, EPSILON).map_err(LinalgError::UtilsError)? {
                    return Ok(false);   // Return the result with no error.
                }
            }
        }

        Ok(true)    // Return the result with no error.
    }

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

    pub fn is_orthogonal(self: &Self) -> Result<bool, LinalgError> {
        // Check if the matrix is square.
        if !self.is_square()? {
            // The matrix is not square so can't be orthogonal.
            return Ok(false);   // Return false with no error.
        }

        let n = self.rows;

        let determinant_abs = fabsf(self.det()?);   // Calculate the absolute value of the determinant of the matrix.

        // Check if the determinant is equal to -1 or 1.
        if !close_all(determinant_abs, 1.0_f32, EPSILON).map_err(LinalgError::UtilsError)? {
            // The absolute value of the determinant is different from 1 so can't be orthogonal.
            return Ok(false);   // Return false wwith no error.
        }

        // Create a copy of the actual matrix and transpose it.
        let mut transposed_mat = Self::new();
        transposed_mat.init(n, n)?;
        transposed_mat.copy_from(&self)?;
        transposed_mat.transpose()?;

        // Create the identity matrix.
        let mut identity_matrix = Self::new();
        identity_matrix.init(n, n)?;
        identity_matrix.fill_identity()?;

        let mut result = Self::new();
        result.init(n, n)?;
        result.mul(&self, &transposed_mat)?;

        // Check that the relationship R Tr(R) = I is satisfied.
        if !result.is_equal_to(&identity_matrix, EPSILON)? {
            // The product of the matrix and its transpose is not equal to the identity matrix and so cannot be orthogonal.
            return Ok(false);   // Return false wwith no error.
        }

        Ok(true)    // Return true with no error.
    }

    pub fn is_so3(self: &Self) -> Result<bool, LinalgError> {
        // Check if the matrix is square.
        if !self.is_square()? {
            // The matrix is not square so can't be orthogonal.
            return Ok(false);   // Return false with no error.
        }

        let n = self.rows;

        // Check if the determinant is equal to 1.
        if !close_all(self.det()?, 1.0_f32, EPSILON).map_err(LinalgError::UtilsError)? {
            // The absolute value of the determinant is different from 1 so can't be orthogonal.
            return Ok(false);   // Return false wwith no error.
        }

        // Create a copy of the actual matrix and transpose it.
        let mut transposed_mat = Self::new();
        transposed_mat.init(n, n)?;
        transposed_mat.copy_from(&self)?;
        transposed_mat.transpose()?;

        // Create the identity matrix.
        let mut identity_matrix = Self::new();
        identity_matrix.init(n, n)?;
        identity_matrix.fill_identity()?;

        let mut result = Self::new();
        result.init(n, n)?;
        result.mul(&self, &transposed_mat)?;

        // Check that the relationship R Tr(R) = I is satisfied.
        if !result.is_equal_to(&identity_matrix, EPSILON)? {
            // The product of the matrix and its transpose is not equal to the identity matrix and so cannot be orthogonal.
            return Ok(false);   // Return false wwith no error.
        }

        Ok(true)    // Return true with no error.
    }
}

// This function is used to duplicate/copy a matrix of size m x n.
pub fn copy_from(matrix: &Matrix) -> Result<Matrix, LinalgError> {
    let mut copied_matrix: Matrix = Matrix::new();
    copied_matrix.init(matrix.get_rows()?, matrix.get_cols()?)?;
    copied_matrix.copy_from(&matrix)?;

    Ok(copied_matrix)
}

// This function is used to perform the matrix addition operation of two matrices of size m x n.
pub fn add(matrix1: &Matrix, matrix2: &Matrix) -> Result<Matrix, LinalgError> {
    let mut result_matrix: Matrix = Matrix::new();
    result_matrix.init(matrix1.get_rows()?, matrix1.get_cols()?)?;
    result_matrix.add(&matrix1, &matrix2)?;

    Ok(result_matrix)
}

// This function is used to perform the matrix subtraction operation of two matrices of size m x n.
pub fn sub(matrix1: &Matrix, matrix2: &Matrix) -> Result<Matrix, LinalgError> {
    let mut result_matrix: Matrix = Matrix::new();
    result_matrix.init(matrix1.get_rows()?, matrix1.get_cols()?)?;
    result_matrix.sub(&matrix1, &matrix2)?;

    Ok(result_matrix)
}

// This function is used to perform the matrix multiplication operation on two matrices of sizes m x n and (n x k) respectively.
// Naive implementation, as it is not very efficient for large matrices (for larger matrices, use the "divide and conquer" strategy).
pub fn mul(matrix1: &Matrix, matrix2: &Matrix) -> Result<Matrix, LinalgError> {
    let mut result_matrix: Matrix = Matrix::new();
    result_matrix.init(matrix1.get_rows()?, matrix2.get_cols()?)?;
    result_matrix.mul(&matrix1, &matrix2)?;

    Ok(result_matrix)
}
