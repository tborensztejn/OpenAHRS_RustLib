extern crate utils;

use utils::utils::UtilsError;

pub const M_MAX: usize = 16;            // Maximum number of rows in a matrix and a vector.
pub const N_MAX: usize = 16;            // Maximum number of columns in a matrix.
pub const EPSILON: f32 = 0.0001;        // ...
pub const PI: f32 = 3.141592653589793;  // ...

#[derive(Debug)]
#[derive(PartialEq)]
pub enum LinalgError {
    NotInit,
    AlreadyInit,
    RowsNumberOverflow,
    RowsNumberNull,
    ColsNumberOverflow,
    ColsNumberNull,
    InvalidRow,
    InvalidCol,
    NotSquare,
    NotSameSize,
    UnchangedSize,
    InvalidSize,
    Unsolvable,
    Singular,
    NotPositiveDefinite,
    NotSymetric,
    NotInversible,
    InvalidAlpha,
    QuaternionSizeMismatch, // Remove it from linalg lib.
    UtilsError(UtilsError),
}

// This function is used to check if the rows number is valid (rows number less than or equal to M_MAX).
pub fn is_valid_rows_number(rows: u8) -> Result<(), LinalgError> {
    if rows > M_MAX as u8 {
        Err(LinalgError::RowsNumberOverflow)
    } else if rows == 0 {
        Err(LinalgError::RowsNumberNull)
    } else {
        Ok(())
    }
}

// This function is used to check if the columns number is valid (colums number less than or equal to N_MAX).
pub fn is_valid_cols_number(cols: u8) -> Result<(), LinalgError> {
    if cols > N_MAX as u8 {
        Err(LinalgError::ColsNumberOverflow)
    } else if cols == 0 {
        Err(LinalgError::ColsNumberNull)
    } else {
        Ok(())
    }
}

// This function is used to check if the index is valid (row number less than number of rows).
pub fn is_valid_row(row: u8, rows: u8) -> Result<(), LinalgError> {
    if row >= rows {
        Err(LinalgError::InvalidRow)
    } else {
        Ok(())
    }
}

// This function is used to check if the index is valid (column number less than number of columns).
pub fn is_valid_col(col: u8, cols: u8) -> Result<(), LinalgError> {
    if col >= cols {
        Err(LinalgError::InvalidCol)
    } else {
        Ok(())
    }
}
