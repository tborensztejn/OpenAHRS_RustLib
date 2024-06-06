use crate::matrix::Matrix;
use crate::vector::Vector;
use crate::common:: { EPSILON, LinalgError};
use libm:: { fabsf, sqrtf };

// This function in used to replace all elements of specific row of a matrix with those of a vector.
pub fn set_row(mat: &mut Matrix, vect: &Vector<f32>, row: u8) -> Result<(), LinalgError> {
    // Check that the number of columns in the matrix is the same as the number of rows in the vector.
    if mat.get_cols()? != vect.get_rows()? {
        // The number of columns in the matrix is not the same as the number of rows in the vector.
        return Err(LinalgError::InvalidSize);   // Return an error.
    }

    for col in 0..mat.get_cols()? {
        mat.set_element(row, col, vect.get_element(col)?)?;
    }

    Ok(())
}

// This function in used to replace all elements of specific column of a matrix with those of a vector.
pub fn set_col(mat: &mut Matrix, vect: &Vector<f32>, col: u8) -> Result<(), LinalgError> {
    // Check that the number of rows in the matrix is the same as the number of rows in the vector.
    if mat.get_rows()? != vect.get_rows()? {
        // The number of rows in the matrix is not the same as the number of rows in the vector.
        return Err(LinalgError::InvalidSize);   // Return an error.
    }

    for row in 0..mat.get_rows()? {
        mat.set_element(row, col, vect.get_element(row)?)?;
    }

    Ok(())
}

// This function is used to extract all elements of specific row of a matrix and store them into a vector.
pub fn get_row(mat: &Matrix, vect: &mut Vector<f32>, row: u8) -> Result<(), LinalgError> {
    // Check that the number of columns in the matrix is the same as the number of rows in the vector.
    if mat.get_cols()? != vect.get_rows()? {
        // The number of columns in the matrix is not the same as the number of rows in the vector.
        return Err(LinalgError::InvalidSize);   // Return an error.
    }

    for col in 0..mat.get_cols()? {
        vect.set_element(col, mat.get_element(row, col)?)?;
    }

    Ok(())
}

// This function is used to extract all elements of specific column of a matrix and store them into a vector.
pub fn get_col(mat: &Matrix, vect: &mut Vector<f32>, col: u8) -> Result<(), LinalgError> {
    // Check that the number of rows in the matrix is the same as the number of rows in the vector.
    if mat.get_rows()? != vect.get_rows()? {
        // The number of rows in the matrix is not the same as the number of rows in the vector.
        return Err(LinalgError::InvalidSize);   // Return an error.
    }

    for row in 0..mat.get_rows()? {
        vect.set_element(row, mat.get_element(row, col)?)?;
    }

    Ok(())
}

// This function is used to extract all elements of specific row of a matrix and store them into a new vector.
pub fn row_to_vector(mat: &Matrix, row: u8) -> Result<Vector<f32>, LinalgError> {
    let mut vect: Vector<f32> = Vector::new();
    vect.init(mat.get_cols()?)?;

    for col in 0..mat.get_cols()? {
        vect.set_element(col, mat.get_element(row, col)?)?;
    }

    Ok(vect)
}

// This function is used to extract all elements of specific column of a matrix and store them into a new vector.
pub fn col_to_vector(mat: &Matrix, col: u8) -> Result<Vector<f32>, LinalgError> {
    let mut vect: Vector<f32> = Vector::new();
    vect.init(mat.get_rows()?)?;

    for row in 0..mat.get_rows()? {
        vect.set_element(row, mat.get_element(row, col)?)?;
    }

    Ok(vect)
}

// This function adds a row to a matrix of size (m x n) from a vector of size (n x 1). The result is a matrix of size (m+1 x n).
pub fn add_row(mat: &mut Matrix, vect: &Vector<f32>, row: u8) -> Result<(), LinalgError> {
    let rows = mat.get_rows()? + 1;
    let cols = mat.get_cols()?;

    mat.set_rows(rows)?;

    for m in (row + 1..rows).rev() {
        for col in 0..cols {
            mat.set_element(m, col, mat.get_element(m - 1, col)?)?;
        }
    }

    set_row(mat, &vect, row)?;

    Ok(())
}

// This function adds a column to a matrix of size (m x n) from a vector of size (m x 1). The result is a matrix of size (m x n+1).
pub fn add_col(mat: &mut Matrix, vect: &Vector<f32>, col: u8) -> Result<(), LinalgError> {
    let rows = mat.get_rows()?;
    let cols = mat.get_cols()? + 1;

    mat.set_cols(cols)?;

    /*
    for row in 0..rows {
        for n in (col + 1..cols).rev() {
            mat.set_element(row, n, mat.get_element(row, n - 1)?)?;
        }
    }
    */

    for n in (col + 1..cols).rev() {
        for row in 0..rows {
            mat.set_element(row, n, mat.get_element(row, n - 1)?)?;
        }
    }

    set_col(mat, &vect, col)?;

    Ok(())
}

// This function is used to convert a vector into a matrix in order to perform matrix operations.
pub fn vector_to_matrix(vect: &Vector<f32>) -> Result<Matrix, LinalgError> {
    let rows = vect.get_rows()?;
    let mut mat = Matrix::new();
    mat.init(rows, 1)?;

    /*
    for row in 0..rows {
        let element = vect.get_element(row)?;
        mat.set_element(row, 0, element)?;
    }
    */

    set_col(&mut mat, &vect, 0)?;

    Ok(mat)
}

/*
pub fn del_row(mat: &mut Matrix, row: u8) -> Result<Matrix, LinalgError> {
    // Add some code here.
}

pub fn del_column(mat: &mut Matrix, col: u8) -> Result<Matrix, LinalgError> {
    // Add some code here.
}

pub fn swap_rows(mat: &mut Matrix, row1: u8, row2: u8) -> Result<(), LinalgError> {
    // Add some code here. Use temporary vector to do the swipe.
}

pub fn swap_cols(mat: &mut Matrix, col1: u8, col2: u8) -> Result<(), LinalgError> {
    // Add some code here. Use temporary vector to do the swipe.
}
*/

pub fn lup(mat: &Matrix) -> Result<(Matrix, Vector<u8>), LinalgError> {
    // Create the LU matrix.
    let mut lu: Matrix = Matrix::new();
    lu.init(mat.get_rows()?, mat.get_cols()?)?;
    lu.copy_from(&mat)?;

    // Create the pivot vector.
    let mut p: Vector<u8> = Vector::new();
    p.init(mat.get_rows()?)?;

    for row in 0..mat.get_rows()? {
        p.set_element(row, row)?;
    }

    for row in 0..(mat.get_rows()? - 1) {
        let mut row_max = row;

        for col in (row + 1)..mat.get_cols()? {
            if lu.get_element(p.get_element(col)?, row)? > lu.get_element(p.get_element(row_max)?, 0)? {
                row_max = col;
            }
        }

        let tmp = p.get_element(row)?;
        p.set_element(row, p.get_element(row_max)?)?;
        p.set_element(row_max, tmp)?;

        // Check if the matrix is singular (up to tolerance).
        if fabsf(lu.get_element(p.get_element(row)?, row)?) < EPSILON {
            // The matrix is singular (up to tolerance).
            return Err(LinalgError::Singular);  // Return an error.
        }

        for col in (row + 1)..mat.get_rows()? {
            let element = lu.get_element(p.get_element(col)?, row)? / lu.get_element(p.get_element(row)?, row)?;
            lu.set_element(p.get_element(col)?, row, element)?;

            for k in (row + 1)..mat.get_rows()? {
                let mut element = lu.get_element(p.get_element(col)?, k)?;
                element = element - lu.get_element(p.get_element(row)?, k)? * lu.get_element(p.get_element(col)?, row)?;
                lu.set_element(p.get_element(col)?, k, element)?;
            }
        }
    }

    Ok((lu, p)) // Return the LU matrix and the pivot vector with no error.
}

pub fn solve(mat: &Matrix, b: &Vector<f32>, p: &Vector<u8>) -> Result<Vector<f32>, LinalgError> {
    // Check that the matrix is square.
    if !mat.is_square()? {
        // The matrix is not square.
        return Err(LinalgError::NotSquare); // Return an error.
    }

    let mut x: Vector<f32> = Vector::new();
    x.init(mat.get_rows()?)?;

    // Forward substitution with pivoting.
    for row in 0..mat.get_rows()? {
        x.set_element(row, b.get_element(p.get_element(row)?)?)?;

        for col in 0..row {
            x.set_element(row, x.get_element(row)? - mat.get_element(p.get_element(row)?, col)? * x.get_element(col)?)?;
        }
    }

    // Backward substitution with pivoting.
    for row in (0..mat.get_rows()?).rev() {
        for col in (row + 1)..mat.get_rows()? {
            x.set_element(row, x.get_element(row)? - mat.get_element(p.get_element(row)?, col)? * x.get_element(col)?)?;
        }

        if fabsf(mat.get_element(p.get_element(row)?, row)?) > EPSILON {
            x.set_element(row, x.get_element(row)? / mat.get_element(p.get_element(row)?, row)?)?;
        } else {
            // Can't be solved.
            return Err(LinalgError::Unsolvable);    // Return an error.
        }
    }

    Ok(x)
}

pub fn eigen(mat: &Matrix, niter: u16, shifted: bool) -> Result<(Vector<f32>, Matrix), LinalgError> {
    // Check that the matrix is square.
    if !mat.is_square()? {
        // The matrix is not square.
        return Err(LinalgError::NotSquare); // Return an error.
    }

    let m = mat.get_rows()?;

    /* Create a copy of the input matrix to avoid modifying it. */
    let mut a: Matrix = Matrix::new();  // Create a new matrix.
    a.init(m, m)?;                      // Initialize the new matrix with the same size as the input matrix.
    a.copy_from(&mat)?;                 // Copy the input matrix into the new one.

    /* Create an identity matrix I. */
    let mut i: Matrix = Matrix::new();  // Create a new matrix.
    i.init(m, m)?;                      // Initialize the new matrix with the same size as the input matrix.
    i.fill_identity()?;                 // Fill in the identity matrix.

    /* Create an orthogonal Q matrix to be used for QR decomposition. */
    let mut q: Matrix = Matrix::new();  // Create a new matrix.
    q.init(m, m)?;                      // Initialize the new matrix with the same size as the input matrix.

    /* Create an upper triangular R matrix to be used for QR decomposition. */
    let mut r: Matrix = Matrix::new();  // Create a new matrix.
    r.init(m, m)?;                      // Initialize the new matrix with the same size as the input matrix.

    /* Create a matrix containing the calculated eigenvectors. */
    let mut eigenvectors: Matrix = Matrix::new();   // Create a new matrix.
    eigenvectors.init(m, m)?;                       // Initialize the new matrix with the same size as the input matrix.
    eigenvectors.fill_identity()?;                  // Fill in the identity matrix.

    /* Create a vector containing the calculated eigenvalues. */
    let mut eigenvalues: Vector<f32> = Vector::new();
    eigenvalues.init(m)?;

    /* Create a vector containing the eigenvalues calculated in the previous loop. */
    let mut previous_eigenvalues: Vector<f32> = Vector::new();
    previous_eigenvalues.init(m)?;

    /* Create a vector containing the variation of calculated eigenvalues. */
    let mut delta_eigenvalues: Vector<f32> = Vector::new();
    delta_eigenvalues.init(m)?;

    for _iter in 0..niter {
        let mut tmp: Matrix = Matrix::new();    // Create a new matrix.
        tmp.init(m, m)?;                        // Initialize the new matrix with the same size as the input matrix.

        if shifted {
            // Apply Wilkinson shift method to determine the shift.
            let a_ = a.get_element(m - 2, m - 2)?;
            let b = a.get_element(m - 2, m - 1)?;
            let c = a.get_element(m - 1, m - 1)?;

            let delta = ((a_ - c) / 2.0) * ((a_ - c) / 2.0) + b * b;

            #[allow(unused_assignments)]
            let mut s: f32 = 0.0;

            if a_ < c {
                s = c - delta / (fabsf(b) + sqrtf(delta));
            } else {
                s = c + delta / (fabsf(b) + sqrtf(delta));
            }

            //Q, R = qr_decomposition(A - s*I)
            //A = R*Q + s*I

            // Apply QR decomposition with shift.
            i.mul_by_scalar(s)?;
            tmp.sub(&a, &i)?;
            a.copy_from(&tmp)?;
            let (q_tmp, r_tmp) = a.qr_decomp()?;
            q.copy_from(&q_tmp)?;
            r.copy_from(&r_tmp)?;

            // Perform the matrix product of the R matrix and the Q matrix with shift.
            tmp.mul(&r, &q)?;
            a.add(&tmp, &i)?;
            i.fill_identity()?;
        } else {
            // Apply regular QR decomposition.
            let (q_tmp, r_tmp) = a.qr_decomp()?;
            q.copy_from(&q_tmp)?;
            r.copy_from(&r_tmp)?;
            // Perform the matrix product of the R matrix and the Q matrix.
            a.mul(&r, &q)?;
        }

        // Find eigenvectors.
        tmp.mul(&eigenvectors, &q)?;
        eigenvectors.copy_from(&tmp)?;

        // If the algorithm has converged before reaching the maximum number of iterations, no error.
        // If the maximum number of iterations is reached before converging, error.

        // Extract the eigenvalues from the diagonal of the matrix.
        for k in 0..m {
            eigenvalues.set_element(k, a.get_element(k, k)?)?;
        }

        // Calculate the relative variation in eigenvalues.
        delta_eigenvalues.sub(&eigenvalues, &previous_eigenvalues)?;
        let relative_variation = delta_eigenvalues.calculate_norm()? / eigenvalues.calculate_norm()?;

        // Check for convergence.
        if relative_variation < EPSILON {
            break;
        } else {
            // Update the previous eigenvalues for the next iteration.
            for k in 0..m {
                eigenvalues.set_element(k, 0.0)?;
                previous_eigenvalues.set_element(k, a.get_element(k, k)?)?;
            }
        }
    }

    Ok((eigenvalues, eigenvectors))
}

/*
pub fn linsolve_lup() -> Result<(Vector<f32>, Matrix), LinalgError> {

}
*/
