extern crate utils;

use utils::utils::UtilsError;

pub const M_MAX: usize = 16;            /// Maximum number of rows in a matrix and a vector.
pub const N_MAX: usize = 16;            /// Maximum number of columns in a matrix.
pub const EPSILON: f32 = 0.0001;        /// Constant value representing a minimal error in numerical analysis.
pub const PI: f32 = 3.141592653589793;  /// Approximate value of constant π.
pub const E: f32 = 2.718281828459045;   /// Approximate value of Euler's number (aka exponential function base or Napier's constant).

//pub type Result<T> = core::result::Result<T, LinalgError>;

#[derive(Debug)]
#[derive(PartialEq)]
pub enum LinalgError {
    NotInit,                /// This error occurs when an attempt is made to manipulate an uninitialised vector or matrix.
    AlreadyInit,            /// This error occurs when an attempt is made to initialise a matrix that has already been initialised.
    RowsNumberOverflow,     /// This error occurs when the number of rows is greater than the maximum number.
    RowsNumberNull,         /// This error occurs when the number of rows is zero.
    ColsNumberOverflow,     /// This error occurs when the number of columns is greater than the maximum number.
    ColsNumberNull,         /// This error occurs when the number of columns is zero.
    InvalidRow,             /// This error occurs when an attempt is made to access a non-existent row in a vector or matrix.
    InvalidCol,             /// This error occurs when an attempt is made to access a non-existent column in a matrix.
    NotSquare,              /// This error occurs when an operation that applies exclusively to a square matrix is applied to a non-square matrix.
    NotSameSize,            /// This error occurs when an operation on a vector or matrix that requires them to have the same dimensions is not carried out.
    UnchangedSize,          /// This error occurs when the number of lines remains unchanged when the dimension is modified.
    InvalidSize,            /// This error occurs when ...
    Unsolvable,             /// This error occurs when a system can't be solved.
    Singular,               /// This error occurs when the matrix is singular.
    NotPositiveDefinite,    /// This error occurs when ...
    NotSymetric,            /// This error occurs when the matrix is not symmetrical.
    NotInversible,          /// This error occurs when the matrix can't be inverted.
    InvalidAlpha,           /// This error occurs when ...
    InvalidDiagonalLength,  /// This error occurs when the specified diagonal to be extracted from a matrix is invalid.
    UtilsError(UtilsError),
}

/// This function is used to check if the rows number is valid (rows number less than or equal to M_MAX).
pub fn is_valid_rows_number(rows: u8) -> Result<(), LinalgError> {
    // Check that the number of rows does not exceed the maximum number of rows (protection against the risk of buffer overflow).
    if rows > M_MAX as u8 {
        // The number of rows exceeds the maximum number of rows (risk of buffer overflow).
        return Err(LinalgError::RowsNumberOverflow);    // Return an error.
    }

    // Check that the number of rows is not zero.
    if rows == 0 {
        // The number of rows can't be zero.
        return Err(LinalgError::RowsNumberNull);    // Return an error.
    }

    Ok(())  // Return no error.
}

/// This function is used to check if the columns number is valid (colums number less than or equal to N_MAX).
pub fn is_valid_cols_number(cols: u8) -> Result<(), LinalgError> {
    // Check that the number of columns does not exceed the maximum number of columns (protection against the risk of buffer overflow).
    if cols > N_MAX as u8 {
        // The number of columns exceeds the maximum number of columns (risk of buffer overflow).
        return Err(LinalgError::ColsNumberOverflow);    // Return an error.
    }

    // Check that the number of columns is not zero.
    if cols == 0 {
        // The number of columns can't be zero.
        return Err(LinalgError::ColsNumberNull);    // Return an error.
    }

    Ok(())  // Return no error.
}

/// This function is used to check if the index is valid (row number less than number of rows).
pub fn is_valid_row(row: u8, rows: u8) -> Result<(), LinalgError> {
    // Check that the row to be accessed is valid.
    if row >= rows {
        // The row to be accessed is invalid or does not exist.
        return Err(LinalgError::InvalidRow);    // Return an error.
    }

    Ok(())  // Return no error.
}

/// This function is used to check if the index is valid (column number less than number of columns).
pub fn is_valid_col(col: u8, cols: u8) -> Result<(), LinalgError> {
    // Check that the column to be accessed is valid.
    if col >= cols {
        // The column to be accessed is invalid or does not exist.
        return Err(LinalgError::InvalidCol);    // Return an error.
    }

    Ok(())  // Return no error.
}
