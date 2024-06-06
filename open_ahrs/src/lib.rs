#![cfg_attr(not(feature = "std"), no_std)]

pub mod common;
pub mod gyroscope;
pub mod accelerometer;
pub mod magnetometer;
pub mod ar;

#[cfg(test)]
#[cfg(feature = "std")]
mod open_ahrs_tests {
    extern crate linalg;
    extern crate quaternion;

    use crate::ar::AR;
    //use crate::common::{SERIES, generate_random_attitudes};
    use crate::common::{CLOSED_FORM, generate_random_attitudes};
    //use linalg::matrix::{Matrix, mul};
    use linalg::matrix::Matrix;
    use linalg::vector::Vector;
    //use linalg::linalg::{vector_to_matrix, col_to_vector, set_col};
    use linalg::linalg::set_col;
    use quaternion::quaternion::Quaternion;

    #[test]
    fn ar_series_method_test() {
        /* Generate random attitudes. */
        let niter: u8 = 10; // Number of iterations.
        let ts: f32 = 0.01;  // Sampling period (s).

        let attitudes: Matrix = generate_random_attitudes(niter + 1).unwrap();
        /*
        let mut attitudes: Matrix = Matrix::new();
        attitudes.init(4, 11).unwrap();

        let mut attitude: Quaternion = Quaternion::new().unwrap();

        // Initial attitude q[t0].
        //attitude.fill(0.007_f32, 0.598_f32, 0.543_f32, 0.590_f32).unwrap();
        attitude.fill(0.8616101_f32, 0.0142137_f32, 0.1483934_f32, -0.485186_f32).unwrap();
        set_col(&mut attitudes, &attitude.get_vect().unwrap(), 0).unwrap();

        // Attitude at t0 + Δt q[t0 + Δt].
        //attitude.fill(0.0_f32, -0.539_f32, 0.842_f32, -0.033_f32).unwrap();
        attitude.fill(0.9710628_f32, 0.0679105_f32, 0.0951136_f32, -0.2082753_f32).unwrap();
        set_col(&mut attitudes, &attitude.get_vect().unwrap(), 1).unwrap();
        */

        /*
        // Attitude at t0 + 2Δt q[t0 + 2Δt].
        attitude.fill(0.619_f32, 0.360_f32, 0.670_f32, -0.196_f32).unwrap();
        set_col(&mut attitudes, &attitude.get_vect().unwrap(), 2).unwrap();

        // Attitude at t0 + 3Δt q[t0 + 3Δt].
        attitude.fill(0.040_f32, 0.764_f32, -0.548_f32, -0.338_f32).unwrap();
        set_col(&mut attitudes, &attitude.get_vect().unwrap(), 3).unwrap();

        // Attitude at t0 + 4Δt q[t0 + 4Δt].
        attitude.fill(-0.140_f32, -0.928_f32, -0.206_f32, 0.276_f32).unwrap();
        set_col(&mut attitudes, &attitude.get_vect().unwrap(), 4).unwrap();

        // Attitude at t0 + 5Δt q[t0 + 5Δt].
        attitude.fill(0.332_f32, 0.725_f32, 0.476_f32, 0.371_f32).unwrap();
        set_col(&mut attitudes, &attitude.get_vect().unwrap(), 5).unwrap();

        // Attitude at t0 + 6Δt q[t0 + 6Δt].
        attitude.fill(0.667_f32, 0.064_f32, 0.198_f32, -0.715_f32).unwrap();
        set_col(&mut attitudes, &attitude.get_vect().unwrap(), 6).unwrap();

        // Attitude at t0 + 7Δt q[t0 + 7Δt].
        attitude.fill(0.420_f32, 0.711_f32, -0.119_f32, -0.552_f32).unwrap();
        set_col(&mut attitudes, &attitude.get_vect().unwrap(), 7).unwrap();

        // Attitude at t0 + 8Δt q[t0 + 8Δt].
        attitude.fill(0.212_f32, -0.174_f32, -0.249_f32, 0.929_f32).unwrap();
        set_col(&mut attitudes, &attitude.get_vect().unwrap(), 8).unwrap();

        // Attitude at t0 + 9Δt q[t0 + 9Δt].
        attitude.fill(-0.539_f32, 0.078_f32, -0.579_f32, -0.606_f32).unwrap();
        set_col(&mut attitudes, &attitude.get_vect().unwrap(), 9).unwrap();

        // Attitude at t0 + 10Δt q[t0 + 10Δt].
        attitude.fill(-0.423_f32, -0.552_f32, 0.469_f32, -0.545_f32).unwrap();
        set_col(&mut attitudes, &attitude.get_vect().unwrap(), 10).unwrap();
        */

        print!("\nAttitudes:\n");
        attitudes.print().unwrap();

        let mut angular_rates: Matrix = Matrix::new();
        angular_rates.init(3, niter).unwrap();
        angular_rates.fill(0.0).unwrap();

        for n in 0..niter {
        //for n in 0..1 {
            // Attitude at time t.
            let mut q_now: Quaternion = Quaternion::new().unwrap();
            // Attitude at time t + Δt.
            let mut q_next: Quaternion = Quaternion::new().unwrap();
            // Quaternion derivative.
            let mut q_dot: Quaternion = Quaternion::new().unwrap();

            // Extract quaternion elements from randomnly generated "attitudes" matrix.
            let mut qw = attitudes.get_element(0, n).unwrap();
            let mut qx = attitudes.get_element(1, n).unwrap();
            let mut qy = attitudes.get_element(2, n).unwrap();
            let mut qz = attitudes.get_element(3, n).unwrap();

            q_now.fill(qw, qx, qy, qz).unwrap();

            // Extract quaternion elements from randomnly generated "attitudes" matrix.
            qw = attitudes.get_element(0, n + 1).unwrap();
            qx = attitudes.get_element(1, n + 1).unwrap();
            qy = attitudes.get_element(2, n + 1).unwrap();
            qz = attitudes.get_element(3, n + 1).unwrap();

            q_next.fill(qw, qx, qy, qz).unwrap();

            // Calculate the derivative using formula q_dot = (q_next - q_now) / dt.
            q_dot.sub(&q_next, &q_now).unwrap();
            q_dot.mul_by_scalar(1.0 / ts).unwrap();
            //print!("q_dot (reference):\n");
            //q_dot.print().unwrap(); // REFERENCE RESULT.

            /* Method 1. OK. */
            //Please note that the missing scalar part does not allow you to calculate the derivative of q(t) from w(t).
            /*
            //let mut w: Vector<f32> = Vector::new();           // Angular rates vector.
            //w.init(3).unwrap();
            let mut w: Quaternion = Quaternion::new().unwrap(); // Angular rates qauternion.
            let wx: f32 = q_now.get_qw().unwrap()*q_next.get_qx().unwrap() - q_now.get_qx().unwrap()*q_next.get_qw().unwrap() - q_now.get_qy().unwrap()*q_next.get_qz().unwrap() + q_now.get_qz().unwrap()*q_next.get_qy().unwrap();
            let wy: f32 = q_now.get_qw().unwrap()*q_next.get_qy().unwrap() + q_now.get_qx().unwrap()*q_next.get_qz().unwrap() - q_now.get_qy().unwrap()*q_next.get_qw().unwrap() - q_now.get_qz().unwrap()*q_next.get_qx().unwrap();
            let wz: f32 = q_now.get_qw().unwrap()*q_next.get_qz().unwrap() - q_now.get_qx().unwrap()*q_next.get_qy().unwrap() + q_now.get_qy().unwrap()*q_next.get_qx().unwrap() - q_now.get_qz().unwrap()*q_next.get_qw().unwrap();
            //w.set_element(0, wx).unwrap();
            //w.set_element(1, wy).unwrap();
            //w.set_element(2, wz).unwrap();
            w.fill(0.0_f32, wx, wy, wz).unwrap();
            w.mul_by_scalar(2.0_f32 / ts).unwrap();
            w.print().unwrap();
            */

            /* Method 2. OK. */
            // Calculate transformation matrix Q.
            /*
            let mut transformation_matrix: Matrix = Matrix::new();
            transformation_matrix.init(4, 3).unwrap();
            transformation_matrix.set_element(0, 0, -q_now.get_qx().unwrap()).unwrap();
            transformation_matrix.set_element(1, 0, q_now.get_qw().unwrap()).unwrap();
            transformation_matrix.set_element(2, 0, q_now.get_qz().unwrap()).unwrap();
            transformation_matrix.set_element(3, 0, -q_now.get_qy().unwrap()).unwrap();
            transformation_matrix.set_element(0, 1, -q_now.get_qy().unwrap()).unwrap();
            transformation_matrix.set_element(1, 1, -q_now.get_qz().unwrap()).unwrap();
            transformation_matrix.set_element(2, 1, q_now.get_qw().unwrap()).unwrap();
            transformation_matrix.set_element(3, 1, q_now.get_qx().unwrap()).unwrap();
            transformation_matrix.set_element(0, 2, -q_now.get_qz().unwrap()).unwrap();
            transformation_matrix.set_element(1, 2, q_now.get_qy().unwrap()).unwrap();
            transformation_matrix.set_element(2, 2, -q_now.get_qx().unwrap()).unwrap();
            transformation_matrix.set_element(3, 2, q_now.get_qw().unwrap()).unwrap();

            // Make a copy of the matrix Q to perform modifications on it.
            let mut transformation_matrix_copy: Matrix = Matrix::new();
            transformation_matrix_copy.init(transformation_matrix.get_rows().unwrap(), transformation_matrix.get_cols().unwrap()).unwrap();
            transformation_matrix_copy.copy_from(&transformation_matrix).unwrap();

            // Calculate angular rate vector using formula w = 2Q^T q_dot
            transformation_matrix_copy.transpose().unwrap();         // Transpose the matrix Q.
            transformation_matrix_copy.mul_by_scalar(2.0).unwrap();  // Multiply each element by 2.
            // Angular rate vector at time t.
            let w: Vector<f32> = col_to_vector(&mul(&transformation_matrix_copy, &vector_to_matrix(&q_dot.get_vect().unwrap()).unwrap()).unwrap(), 0).unwrap();
            set_col(&mut angular_rates, &w, n).unwrap();
            //w.print().unwrap();

            /* Used to find derivative of q(t) from w(t). */
            transformation_matrix.mul_by_scalar(1.0_f32 / 2.0_f32).unwrap();    // Divide each element by 2.
            let w: Matrix = vector_to_matrix(&w).unwrap();
            let q_dot: Matrix = mul(&transformation_matrix, &w).unwrap();
            print!("q_dot (calculated):\n");
            q_dot.print().unwrap();
            */






            /* Method 3. OK. */
            let mut w: Quaternion = Quaternion::new().unwrap();
            let mut q_now_conj: Quaternion = Quaternion::new().unwrap();
            q_now_conj.copy_from(&q_now).unwrap();
            q_now_conj.conjugate().unwrap();
            q_now_conj.mul_by_scalar(2.0_f32).unwrap();
            w.mul(&q_now_conj, &q_dot).unwrap();

            let wx = w.get_qx().unwrap();
            let wy = w.get_qy().unwrap();
            let wz = w.get_qz().unwrap();

            let mut w_vect: Vector<f32> = Vector::new();
            w_vect.init(3).unwrap();
            w_vect.set_element(0, wx).unwrap();
            w_vect.set_element(1, wy).unwrap();
            w_vect.set_element(2, wz).unwrap();

            set_col(&mut angular_rates, &w_vect, n).unwrap();

            /* Used to find derivative of q(t) from w(t). */
            q_dot.mul(&q_now, &w).unwrap();
            q_dot.mul_by_scalar(1.0_f32 / 2.0_f32).unwrap();
            //print!("q_dot (calculated):\n");
            //q_dot.print().unwrap();
        }

        print!("Angular rates:\n");
        angular_rates.print().unwrap();

        let mut ar_filter: AR = AR::new().unwrap();

        // Extract initial quaternion elements.
        let qw = attitudes.get_element(0, 0).unwrap();
        let qx = attitudes.get_element(1, 0).unwrap();
        let qy = attitudes.get_element(2, 0).unwrap();
        let qz = attitudes.get_element(3, 0).unwrap();

        ar_filter.init(
            // Initial orientation.
            qw, qx, qy, qz,
            // Gyroscope settings.
            1.0_f32, 1.0_f32, 1.0_f32,
            0.0_f32, 0.0_f32,
            0.0_f32, 0.0_f32,
            0.0_f32, 0.0_f32,
            0.0_f32, 0.0_f32, 0.0_f32,
            ts,
            CLOSED_FORM,
            1_u8
        ).unwrap();

        for n in 0..niter {
        //for n in 0..1 {
            let gx = angular_rates.get_element(0, n).unwrap();
            let gy = angular_rates.get_element(1, n).unwrap();
            let gz = angular_rates.get_element(2, n).unwrap();

            ar_filter.update(gx, gy, gz).unwrap();
            ar_filter.print_attitude().unwrap();
        }
    }
}
