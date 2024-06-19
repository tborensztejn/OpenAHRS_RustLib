#![cfg_attr(not(feature = "std"), no_std)]

pub mod common;
pub mod gyrometer;
pub mod accelerometer;
pub mod magnetometer;
pub mod ar;
pub mod aqua;

#[cfg(test)]
#[cfg(feature = "std")]
mod open_ahrs_tests {
    extern crate linalg;
    extern crate quaternion;

    use crate::ar::AR;
    use crate::gyrometer::GyrometerConfig;
    //use crate::aqua::AQUA;
    use crate::common::{NumericalIntegrationMethod as NIM, generate_random_attitudes};
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

        print!("\nAttitudes:\n");
        attitudes.print().unwrap();

        let mut angular_rates: Matrix = Matrix::new();
        angular_rates.init(3, niter).unwrap();
        angular_rates.fill(0.0).unwrap();

        for n in 0..niter {
            let mut q_now: Vector<f32> = Vector::new();     // Attitude at time t.
            q_now.init(4).unwrap();
            let mut q_next: Vector<f32> = Vector::new();    // Attitude at time t + Δt.
            q_next.init(4).unwrap();
            let mut q_dot: Vector<f32> = Vector::new();     // Quaternion derivative.
            q_dot.init(4).unwrap();

            // Extract quaternion elements from randomnly generated "attitudes" matrix.
            let mut qw = attitudes.get_element(0, n).unwrap();
            let mut qx = attitudes.get_element(1, n).unwrap();
            let mut qy = attitudes.get_element(2, n).unwrap();
            let mut qz = attitudes.get_element(3, n).unwrap();

            q_now.fillq(qw, qx, qy, qz).unwrap();

            // Extract quaternion elements from randomnly generated "attitudes" matrix.
            qw = attitudes.get_element(0, n + 1).unwrap();
            qx = attitudes.get_element(1, n + 1).unwrap();
            qy = attitudes.get_element(2, n + 1).unwrap();
            qz = attitudes.get_element(3, n + 1).unwrap();

            q_next.fillq(qw, qx, qy, qz).unwrap();

            // Calculate the derivative using formula q_dot = (q_next - q_now) / dt.
            q_dot.sub(&q_next, &q_now).unwrap();
            q_dot.mul_by_scalar(1.0_f32 / ts).unwrap();
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
            let mut w: Vector<f32> = Vector::new();
            w.init(4).unwrap();
            let mut q_now_conj: Vector<f32> = Vector::new();
            q_now_conj.init(4).unwrap();
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

        let default_gyrometer_config = GyrometerConfig::default();

        /*
        let custom_config = GyrometerConfig {
            x_axis_scale_correction: 1.1,
            y_axis_scale_correction: 1.2,
            z_axis_scale_correction: 1.3,
            xy_axes_misalignment_correction: 0.01,
            xz_axes_misalignment_correction: 0.02,
            yx_axes_misalignment_correction: 0.01,
            yz_axes_misalignment_correction: 0.02,
            zx_axes_misalignment_correction: 0.01,
            zy_axes_misalignment_correction: 0.02,
            x_axis_static_bias: 0.1,
            y_axis_static_bias: 0.2,
            z_axis_static_bias: 0.3,
        };

        let partial_custom_config = GyrometerConfig {
            x_axis_scale_correction: 1.1,
            y_axis_scale_correction: 1.2,
            ..GyrometerConfig::default()
        };
        */

        ar_filter.init(
            // Initial orientation.
            qw, qx, qy, qz,
            // Gyrometer configuration.
            default_gyrometer_config,
            ts,
            NIM::Euler,
            3_u8
        ).unwrap();

        for n in 0..niter {
            let gx = angular_rates.get_element(0, n).unwrap();
            let gy = angular_rates.get_element(1, n).unwrap();
            let gz = angular_rates.get_element(2, n).unwrap();

            ar_filter.update(gx, gy, gz).unwrap();
            ar_filter.print_attitude().unwrap();
        }

        /*
        let mut quat: Vector<f32> = Vector::new();
        quat.init(3).unwrap();
        //quat.fill(0.5, 5.5, 56.2, 89.8).unwrap();
        //Quat::fillq(&mut quat, 0.5, 0.5, 0.5, 0.5).unwrap();   // Appel non implicite.
        //quat.set_qw(0.5).unwrap();  // Appel implicite de la fonction.
        quat.fillq(0.5, 0.5, 0.5, 0.5).unwrap();
        */
    }
}
