//#![no_std]
#![cfg_attr(not(feature = "std"), no_std)]

pub mod common;
pub mod matrix;
pub mod vector;
pub mod linalg;

#[cfg(test)]
mod matrix_tests {
    use crate::matrix::Matrix;
    use crate::vector::Vector;
    use crate::common:: {M_MAX, N_MAX, LinalgError};
    //use crate::linalg::eigen;
    use crate::linalg:: {set_row, set_col, add_row, add_col};

    /// This test function is used to check that there are no errors in the correct creation of a matrix. No error expected.
    #[test]
    //#[ignore]
    fn init_test_1() {
        let rows: u8 = M_MAX as u8;
        let cols: u8 = N_MAX as u8;

        let mut mat: Matrix = Matrix::new();
        let result = mat.init(rows, cols);

        assert!(result.is_ok(), "Unexpected result: {:?}.", result);
    }

    /// This test function is used to check that it is not possible to create a matrix with zero rows. Expected error.
    #[test]
    //#[ignore]
    fn init_test_2() {
        let rows: u8 = 0;
        let cols: u8 = N_MAX as u8;

        let mut mat: Matrix = Matrix::new();
        let result = mat.init(rows, cols);

        assert!(result.is_err(), "Expected an error, but got Ok.");
        assert_eq!(result.unwrap_err(), LinalgError::RowsNumberNull, "Unexpected error.");
    }

    /// This test function is used to check that it is not possible to create a matrix with zero columns. Expected error.
    #[test]
    //#[ignore]
    fn init_test_3() {
        let rows: u8 = M_MAX as u8;
        let cols: u8 = 0;

        let mut mat: Matrix = Matrix::new();
        let result = mat.init(rows, cols);

        assert!(result.is_err(), "Expected an error, but got Ok.");
        assert_eq!(result.unwrap_err(), LinalgError::ColsNumberNull, "Unexpected error.");
    }

    /// This test function is used to check that it is not possible to create a matrix with a number of rows greater than M_MAX. Expected error.
    #[test]
    //#[ignore]
    fn init_test_4() {
        let rows: u8 = M_MAX as u8 + 1;
        let cols: u8 = N_MAX as u8;

        let mut mat: Matrix = Matrix::new();
        let result = mat.init(rows, cols);

        assert!(result.is_err(), "Expected an error, but got Ok.");
        assert_eq!(result.unwrap_err(), LinalgError::RowsNumberOverflow, "Unexpected error.");
    }

    /// This test function is used to check that it is not possible to create a matrix with a number of columns greater than N_MAX. Expected error.
    #[test]
    //#[ignore]
    fn init_test_5() {
        let rows: u8 = M_MAX as u8;
        let cols: u8 = N_MAX as u8 + 1;

        let mut mat: Matrix = Matrix::new();
        let result = mat.init(rows, cols);

        assert!(result.is_err(), "Expected an error, but got Ok.");
        assert_eq!(result.unwrap_err(), LinalgError::ColsNumberOverflow, "Unexpected error.");
    }

    /// This test function is used to check that it is not possible to initialize a matrix that has already been initialized. Expected error.
    #[test]
    //#[ignore]
    fn init_test_6() {
        let rows: u8 = M_MAX as u8;
        let cols: u8 = N_MAX as u8;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();
        let result = mat.init(rows, cols);

        assert!(result.is_err(), "Expected an error, but got Ok.");
        assert_eq!(result.unwrap_err(), LinalgError::AlreadyInit, "Unexpected error.");
    }

    /// This test function is used to check that there are no errors when a matrix element is correctly modified. No error expected.
    #[test]
    //#[ignore]
    fn set_element_test_1() {
        let rows: u8 = M_MAX as u8;
        let cols: u8 = N_MAX as u8;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();

        for row in 0..rows {
            for col in 0..cols {
                let result = mat.set_element(row, col, 1.0);

                assert!(result.is_ok(), "Unexpected result: {:?}.", result);
            }
        }
    }

    /// This test function is used to check that it is not possible to modify an element of an uninitialized matrix. Expected error.
    #[test]
    //#[ignore]
    fn set_element_test_2() {
        let mut mat: Matrix = Matrix::new();

        let result = mat.set_element(0_u8, 0_u8, 1.0);

        assert!(result.is_err(), "Expected an error, but got Ok.");
        assert_eq!(result.unwrap_err(), LinalgError::NotInit, "Unexpected error.");
    }

    /// This test function is used to check that it is not possible to modify an element in a non-existent row of an initialized matrix. Expected error.
    #[test]
    //#[ignore]
    fn set_element_test_3() {
        let rows: u8 = M_MAX as u8;
        let cols: u8 = N_MAX as u8;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();

        let result = mat.set_element(rows, 0_u8, 1.0);

        assert!(result.is_err(), "Expected an error, but got Ok.");
        assert_eq!(result.unwrap_err(), LinalgError::InvalidRow, "Unexpected error.");
    }

    /// This test function is used to check that it is not possible to modify an element in a non-existent column of an initialized matrix. Expected error.
    #[test]
    //#[ignore]
    fn set_element_test_4() {
        let rows: u8 = M_MAX as u8;
        let cols: u8 = N_MAX as u8;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();

        let result = mat.set_element(0_u8, cols, 1.0);

        assert!(result.is_err(), "Expected an error, but got Ok.");
        assert_eq!(result.unwrap_err(), LinalgError::InvalidCol, "Unexpected error.");
    }

    /// This test function is used to check that there are no errors when correctly accessing an element in a matrix. No error expected.
    #[test]
    //#[ignore]
    fn get_element_test_1() {
        let rows: u8 = M_MAX as u8;
        let cols: u8 = N_MAX as u8;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();

        for row in 0..rows {
            for col in 0..cols {
                let result = mat.set_element(row, col, 1.0);

                assert!(result.is_ok(), "Unexpected result: {:?}.", result);
                assert_eq!(mat.get_element(row, col).unwrap(), 1.0, "Unexpected element value.");
            }
        }
    }

    ///
    #[test]
    //#[ignore]
    fn get_element_test_2() {
        let mat: Matrix = Matrix::new();

        let result = mat.get_element(0_u8, 0_u8);

        assert!(result.is_err(), "Expected an error, but got Ok.");
        assert_eq!(result.unwrap_err(), LinalgError::NotInit, "Unexpected error.");
    }

    #[test]
    //#[ignore]
    fn get_element_test_3() {
        let rows: u8 = M_MAX as u8;
        let cols: u8 = N_MAX as u8;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();

        let result = mat.get_element(rows, 0_u8);

        assert!(result.is_err(), "Expected an error, but got Ok.");
        assert_eq!(result.unwrap_err(), LinalgError::InvalidRow, "Unexpected error.");
    }

    #[test]
    //#[ignore]
    fn get_element_test_4() {
        let rows: u8 = M_MAX as u8;
        let cols: u8 = N_MAX as u8;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();

        let result = mat.get_element(0_u8, cols);

        assert!(result.is_err(), "Expected an error, but got Ok.");
        assert_eq!(result.unwrap_err(), LinalgError::InvalidCol, "Unexpected error.");
    }

    #[test]
    //#[ignore]
    fn get_rows_test_1() {
        let rows: u8 = M_MAX as u8;
        let cols: u8 = N_MAX as u8;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();

        let result = mat.get_rows();

        assert!(result.is_ok(), "Unexpected result: {:?}", result);
        assert_eq!(result.unwrap(), rows, "Unexpected number of rows.");
    }

    #[test]
    //#[ignore]
    fn get_rows_test_2() {
        let mat: Matrix = Matrix::new();

        let result = mat.get_rows();

        assert!(result.is_err(), "Expected an error, but got Ok.");
        assert_eq!(result.unwrap_err(), LinalgError::NotInit, "Unexpected error.");
    }

    #[test]
    //#[ignore]
    fn get_cols_test_1() {
        let rows: u8 = M_MAX as u8;
        let cols: u8 = N_MAX as u8;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();

        let result = mat.get_cols();

        assert!(result.is_ok(), "Unexpected result: {:?}", result);
        assert_eq!(result.unwrap(), cols, "Unexpected number of rows.");
    }

    #[test]
    //#[ignore]
    fn get_cols_test_2() {
        let mat: Matrix = Matrix::new();

        let result = mat.get_cols();

        assert!(result.is_err(), "Expected an error, but got Ok.");
        assert_eq!(result.unwrap_err(), LinalgError::NotInit, "Unexpected error.");
    }

    #[test]
    //#[ignore]
    fn fill_matrix_test() {
        let rows: u8 = M_MAX as u8;
        let cols: u8 = N_MAX as u8;
        let value_to_fill: f32 = 1.0;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();

        let result = mat.fill(value_to_fill);

        assert!(result.is_ok(), "Unexpected result: {:?}", result);

        // Check that all elements in the matrix are filled with the expected value.
        for row in 0..rows {
            for col in 0..cols {
                assert_eq!(mat.get_element(row, col).unwrap(), value_to_fill, "Unexpected element value.");
            }
        }
    }

    #[test]
    //#[ignore]
    fn is_square_test_1() {
        let rows: u8 = 3;
        let cols: u8 = 3;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();

        let result = mat.is_square();

        assert!(result.is_ok(), "Unexpected result: {:?}.", result);
        assert_eq!(result.unwrap(), true, "Expected a square matrix.");
    }

    #[test]
    //#[ignore]
    fn is_square_test_2() {
        let rows: u8 = 3;
        let cols: u8 = 2;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();

        let result = mat.is_square();

        assert!(result.is_ok(), "Unexpected result: {:?}.", result);
        assert_eq!(result.unwrap(), false, "Expected a non-square matrix.");
    }

    #[test]
    //#[ignore]
    fn is_square_test_3() {
        let mat: Matrix = Matrix::new();

        let result = mat.is_square();

        assert!(result.is_err(), "Expected an error, but got Ok.");
        assert_eq!(result.unwrap_err(), LinalgError::NotInit, "Unexpected error.");
    }

    #[test]
    //#[ignore]
    fn fill_identity_test_1() {
        let rows: u8 = 3;
        let cols: u8 = 3;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();

        let result = mat.fill_identity();

        assert!(result.is_ok(), "Unexpected result: {:?}.", result);

        // Verify that the matrix is an identity matrix.
        for row in 0..rows {
            for col in 0..cols {
                if row == col {
                    assert_eq!(mat.get_element(row, col).unwrap(), 1.0, "Unexpected element value.");
                } else {
                    assert_eq!(mat.get_element(row, col).unwrap(), 0.0, "Unexpected element value.");
                }
            }
        }
    }

    #[test]
    //#[ignore]
    fn fill_identity_test_2() {
        let rows: u8 = 3;
        let cols: u8 = 4;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();

        let result = mat.fill_identity();

        assert!(result.is_err(), "Expected an error, but got Ok.");
        assert_eq!(result.unwrap_err(), LinalgError::NotSquare, "Unexpected error.");
    }

    #[test]
    //#[ignore]
    fn is_same_size_as_test_1() {
        let rows: u8 = 3;
        let cols: u8 = 3;

        let mut mat1: Matrix = Matrix::new();
        mat1.init(rows, cols).unwrap();

        let mut mat2: Matrix = Matrix::new();
        mat2.init(rows, cols).unwrap();

        let result = mat1.is_same_size_as(&mat2);

        assert!(result.is_ok(), "Unexpected result: {:?}.", result);
        assert_eq!(result.unwrap(), true, "Expected matrices with the same size.");
    }

    #[test]
    //#[ignore]
    fn is_same_size_as_test_2() {
        let rows1: u8 = 3;
        let cols1: u8 = 4;

        let mut mat1: Matrix = Matrix::new();
        mat1.init(rows1, cols1).unwrap();

        let rows2: u8 = 3;
        let cols2: u8 = 3;

        let mut mat2: Matrix = Matrix::new();
        mat2.init(rows2, cols2).unwrap();

        let result = mat1.is_same_size_as(&mat2);

        assert!(result.is_ok(), "Unexpected result: {:?}.", result);
        assert_eq!(result.unwrap(), false, "Expected matrices with different sizes.");
    }

    #[test]
    #[ignore]
    fn foo() {
        let rows: u8 = 3;
        let cols: u8 = 3;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();

        let mut vect: Vector<f32> = Vector::new();
        vect.init(cols).unwrap();
        vect.fill(1.0).unwrap();

        #[cfg(feature = "std")]
        mat.print().unwrap();

        set_row(&mut mat, &vect, 1).unwrap();
        #[cfg(feature = "std")]
        mat.print().unwrap();
    }

    #[test]
    #[ignore]
    fn bar() {
        let rows: u8 = 3;
        let cols: u8 = 3;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();
        mat.fill(2.0).unwrap();

        let mut vect: Vector<f32> = Vector::new();
        vect.init(cols).unwrap();
        vect.fill(1.0).unwrap();

        #[cfg(feature = "std")]
        mat.print().unwrap();

        add_row(&mut mat, &vect, 2).unwrap();
        #[cfg(feature = "std")]
        mat.print().unwrap();
    }

    #[test]
    #[ignore]
    fn foofoo() {
        let rows: u8 = 3;
        let cols: u8 = 3;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();

        let mut vect: Vector<f32> = Vector::new();
        vect.init(rows).unwrap();
        vect.fill(1.0).unwrap();

        #[cfg(feature = "std")]
        mat.print().unwrap();

        set_col(&mut mat, &vect, 2).unwrap();
        #[cfg(feature = "std")]
        mat.print().unwrap();
    }

    #[test]
    #[ignore]
    fn foobar() {
        let rows: u8 = 3;
        let cols: u8 = 3;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();
        mat.fill(2.0).unwrap();

        let mut vect: Vector<f32> = Vector::new();
        vect.init(rows).unwrap();
        vect.fill(1.0).unwrap();

        #[cfg(feature = "std")]
        mat.print().unwrap();

        add_col(&mut mat, &vect, 3).unwrap();
        #[cfg(feature = "std")]
        mat.print().unwrap();
    }

    #[test]
    #[ignore]
    fn test1() {
        let rows: u8 = 3;
        let cols: u8 = 3;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();

        /*
        mat.set_element(0, 0, 1.0).unwrap();
        mat.set_element(0, 1, 2.0).unwrap();
        mat.set_element(0, 2, 3.0).unwrap();

        mat.set_element(1, 0, 4.0).unwrap();
        mat.set_element(1, 1, 5.0).unwrap();
        mat.set_element(1, 2, 6.0).unwrap();

        mat.set_element(2, 0, 7.0).unwrap();
        mat.set_element(2, 1, 8.0).unwrap();
        mat.set_element(2, 2, 9.0).unwrap();
        */

        mat.set_element(0, 0, 1.0).unwrap();
        mat.set_element(0, 1, 2.0).unwrap();
        mat.set_element(0, 2, 3.0).unwrap();

        mat.set_element(1, 0, 0.0).unwrap();
        mat.set_element(1, 1, 1.0).unwrap();
        mat.set_element(1, 2, 4.5).unwrap();

        mat.set_element(2, 0, 5.0).unwrap();
        mat.set_element(2, 1, 6.0).unwrap();
        mat.set_element(2, 2, 0.0).unwrap();

        #[cfg(feature = "std")]
        mat.print().unwrap();
        #[cfg(feature = "std")]
        println!("det = {:.3}", mat.det().unwrap());
    }

    #[test]
    //#[ignore]
    fn test2() {
        let rows: u8 = 3;
        let cols: u8 = 3;

        let mut mat: Matrix = Matrix::new();
        mat.init(rows, cols).unwrap();

        mat.set_element(0, 0, 4.0).unwrap();
        mat.set_element(0, 1, -1.0).unwrap();
        mat.set_element(0, 2, 2.0).unwrap();

        mat.set_element(1, 0, -1.0).unwrap();
        mat.set_element(1, 1, 5.0).unwrap();
        mat.set_element(1, 2, -3.0).unwrap();

        mat.set_element(2, 0, 2.0).unwrap();
        mat.set_element(2, 1, -3.0).unwrap();
        mat.set_element(2, 2, 6.0).unwrap();

        let _l = mat.chol().unwrap();

        #[cfg(feature = "std")]
        _l.print().unwrap();
    }
}
