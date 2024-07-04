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
    extern crate libm;
    extern crate quaternion;

    use crate::ar::AR;
    use crate::gyrometer::GyrometerConfig;
    use crate::accelerometer::AccelerometerConfig;
    use crate::magnetometer::MagnetometerConfig;
    use crate::aqua::{AQUA, Mode};
    use crate::common::{NumericalIntegrationMethod as NIM, generate_random_attitudes};
    use linalg::matrix::Matrix;
    use linalg::vector::Vector;
    use quaternion::quaternion::Quaternion;
    use libm::{cosf, sinf};

    #[test]
    fn ar_series_method_test() {
        //const RAG_2_DEG: f32 = 57.2958;     // Conversion factor between radians to degrees.
        const DEG_2_RAD: f32 = 0.0174533;   // Conversion factor between degrees to radians.

        let niter: u8 = 10;     // Number of iterations.
        let ts: f32 = 0.01;     // Sampling period [s].
        let g: f32 = -9.81;     // Gravitational acceleration [m/s²].
        let b: f32 = 48.0;      // Magnetic field magnitude [µT].
        let i: f32 = 35.0;      // Magnetic field dip [°].

        let bx = b*cosf(i*DEG_2_RAD);
        let bz = -b*sinf(i*DEG_2_RAD);

        /*
        let mut acc_global: Vector<f32> = Vector::new();            // Create the acceleration vector described in the global reference frame.
        acc_global.init(3).unwrap();                                // Initialize it.
        // Fill it.
        acc_global.fill(0.0).unwrap();
        acc_global.set_element(2, g).unwrap();

        let mut mag_global: Vector<f32> = Vector::new();            // Create the acceleration vector described in the global reference frame.
        mag_global.init(3).unwrap();                                // Initialize it.
        // Fill it.
        mag_global.set_element(0, b*cosf(i*DEG_2_RAD)).unwrap();
        mag_global.set_element(1, 0.0).unwrap();
        mag_global.set_element(2, -b*sinf(i*DEG_2_RAD)).unwrap();
        */

        let mut acc_global: Vector<f32> = Vector::new();            // Create the acceleration quaternion described in the global reference frame.
        acc_global.init(4).unwrap();                                // Initialize it.
        acc_global.fillq(0.0, 0.0, 0.0, g).unwrap();                // Fill it.

        let mut mag_global: Vector<f32> = Vector::new();            // Create the acceleration quaternion described in the global reference frame.
        mag_global.init(4).unwrap();                                // Initialize it.
        mag_global.fillq(0.0, bx, 0.0, bz).unwrap();                // Fill it.

        let attitudes: Matrix = generate_random_attitudes(niter + 1).unwrap();  // Generate random attitudes.

        print!("\nRandomly generated attitudes:\n\n");

        for n in 0..niter+1 {
            print!("q[{:}Δt]\t", n);
        }

        print!("\n\n");

        attitudes.print().unwrap();

        let mut angular_rates = Matrix::new();              // Create a new matrix to store calculated angular rates from the local frame.
        angular_rates.init(3, 3).unwrap();                  // Initialize it.
        angular_rates.fill(0.0).unwrap();                   // Fill it with zero value.

        let mut gyrometer_measurements = Matrix::new();     // Create a new matrix to store simulated raw measurements of the gyrometer.
        gyrometer_measurements.init(3, niter).unwrap();     // Initialize it.
        gyrometer_measurements.fill(0.0).unwrap();          // Fill it with zero value.

        let mut accelerometer_measurements = Matrix::new(); // Create a new matrix to store simulated raw measurements of the accelerometer.
        accelerometer_measurements.init(3, niter).unwrap(); // Initialize it.
        accelerometer_measurements.fill(0.0).unwrap();      // Fill it with zero value.

        let mut magnetometer_measurements = Matrix::new();  // Create a new matrix to store simulated raw measurements of the magnetometer.
        magnetometer_measurements.init(3, niter).unwrap();  // Initialize it.
        magnetometer_measurements.fill(0.0).unwrap();       // Fill it with zero value.

        for n in 0..niter {
            let mut q_now: Vector<f32> = Vector::new();     // Create a new quaternion to store the attitude at time t.
            q_now.init(4).unwrap();                         // Initialize it.
            let mut q_next: Vector<f32> = Vector::new();    // Create a new quaternion to store the attitude at time t + Δt.
            q_next.init(4).unwrap();                        // Initialize it.
            let mut q_dot: Vector<f32> = Vector::new();     // Create a new quaternion to store the quaternion derivative.
            q_dot.init(4).unwrap();                         // Initialize it.

            // Extract the coordinates of the quaternion representing the attitude at time nt.
            let mut qw = attitudes.get_element(0, n).unwrap();
            let mut qx = attitudes.get_element(1, n).unwrap();
            let mut qy = attitudes.get_element(2, n).unwrap();
            let mut qz = attitudes.get_element(3, n).unwrap();

            q_now.fillq(qw, qx, qy, qz).unwrap();   // Fill it.

            // Extract the coordinates of the quaternion representing the attitude at time nt + Δt.
            qw = attitudes.get_element(0, n + 1).unwrap();
            qx = attitudes.get_element(1, n + 1).unwrap();
            qy = attitudes.get_element(2, n + 1).unwrap();
            qz = attitudes.get_element(3, n + 1).unwrap();

            q_next.fillq(qw, qx, qy, qz).unwrap();  // Fill it.

            // Calculate the derivative using formula q_dot = (q_next - q_now) / dt.
            q_dot.sub(&q_next, &q_now).unwrap();
            q_dot.mul_by_scalar_in_place(1.0 / ts).unwrap();
            //q_dot.print().unwrap();

            /* Method 1. OK. */
            //Please note that the missing scalar part does not allow you to calculate the derivative of q(t) from w(t).
            let mut w: Vector<f32> = Vector::new(); // Create angular rates vector.
            w.init(3).unwrap();                     // Initialize it.

            // Calculate components of the angular rates vector.
            let wx: f32 = q_now.get_qw().unwrap()*q_next.get_qx().unwrap() - q_now.get_qx().unwrap()*q_next.get_qw().unwrap() - q_now.get_qy().unwrap()*q_next.get_qz().unwrap() + q_now.get_qz().unwrap()*q_next.get_qy().unwrap();
            let wy: f32 = q_now.get_qw().unwrap()*q_next.get_qy().unwrap() + q_now.get_qx().unwrap()*q_next.get_qz().unwrap() - q_now.get_qy().unwrap()*q_next.get_qw().unwrap() - q_now.get_qz().unwrap()*q_next.get_qx().unwrap();
            let wz: f32 = q_now.get_qw().unwrap()*q_next.get_qz().unwrap() - q_now.get_qx().unwrap()*q_next.get_qy().unwrap() + q_now.get_qy().unwrap()*q_next.get_qx().unwrap() - q_now.get_qz().unwrap()*q_next.get_qw().unwrap();

            // Fill it.
            w.set_element(0, wx).unwrap();
            w.set_element(1, wy).unwrap();
            w.set_element(2, wz).unwrap();

            w.mul_by_scalar_in_place(2.0 / ts).unwrap();
            angular_rates.set_col(&w, 0).unwrap();
            //w.print().unwrap();

            /* Method 2. OK. */
            // Calculate transformation matrix Q.
            let mut q_mat = Matrix::new();  // Create the Q transformation matrix.
            q_mat.init(4, 3).unwrap();      // Initialize it.

            // Fill it.
            q_mat.set_element(0, 0, -q_now.get_qx().unwrap()).unwrap();
            q_mat.set_element(1, 0, q_now.get_qw().unwrap()).unwrap();
            q_mat.set_element(2, 0, q_now.get_qz().unwrap()).unwrap();
            q_mat.set_element(3, 0, -q_now.get_qy().unwrap()).unwrap();
            q_mat.set_element(0, 1, -q_now.get_qy().unwrap()).unwrap();
            q_mat.set_element(1, 1, -q_now.get_qz().unwrap()).unwrap();
            q_mat.set_element(2, 1, q_now.get_qw().unwrap()).unwrap();
            q_mat.set_element(3, 1, q_now.get_qx().unwrap()).unwrap();
            q_mat.set_element(0, 2, -q_now.get_qz().unwrap()).unwrap();
            q_mat.set_element(1, 2, q_now.get_qy().unwrap()).unwrap();
            q_mat.set_element(2, 2, -q_now.get_qx().unwrap()).unwrap();
            q_mat.set_element(3, 2, q_now.get_qw().unwrap()).unwrap();

            // Make a copy of the matrix Q to perform modifications on it.
            let mut q_mat_copy = q_mat.duplicate().unwrap();

            // Calculate angular rate vector using formula w = 2Q^T q_dot
            q_mat_copy.transpose().unwrap();        // Transpose the matrix Q.
            q_mat_copy.mul_by_scalar(2.0).unwrap(); // Multiply each element by 2.

            // Angular rate vector at time t.
            let w = q_mat_copy.mul_new(&q_dot.convert_to_matrix().unwrap()).unwrap();
            let w = w.col_to_vector(0).unwrap();
            angular_rates.set_col(&w, 1).unwrap();
            //w.print().unwrap();

            /*
            // Used to find derivative of q(t) from w(t).
            q_mat.mul_by_scalar(1.0 / 2.0).unwrap();    // Divide each element by 2.
            let q_dot_ = q_mat.mul_new(&w.convert_to_matrix().unwrap()).unwrap();
            //q_dot_.print().unwrap();
            */

            /* Method 3. OK. */
            let mut q_now_conj = q_now.conjugate_new().unwrap();
            q_now_conj.mul_by_scalar_in_place(2.0).unwrap();
            let w = q_now_conj.mul_new(&q_dot).unwrap();
            let w_ = w.get_vector_part().unwrap();
            angular_rates.set_col(&w_, 2).unwrap();
            //w_.print().unwrap();

            /*
            // Used to find derivative of q(t) from w(t).
            q_dot.mul(&q_now, &w).unwrap();
            q_dot.mul_by_scalar_in_place(1.0 / 2.0).unwrap();
            //q_dot.print().unwrap();
            */

            gyrometer_measurements.set_col(&w_, n).unwrap();
            //angular_rates.print().unwrap();


            // Add some code here.
            let mut acc_local = q_now.mul_new(&acc_global).unwrap();
            acc_local.mul(&acc_local.duplicate().unwrap(), &q_now.conjugate_new().unwrap()).unwrap();
            let acc_local = acc_local.get_vector_part().unwrap();
            accelerometer_measurements.set_col(&acc_local, n).unwrap();
            //acc_local.print().unwrap();

            let mut mag_local = q_now.mul_new(&mag_global).unwrap();
            mag_local.mul(&mag_local.duplicate().unwrap(), &q_now.conjugate_new().unwrap()).unwrap();
            let mag_local = mag_local.get_vector_part().unwrap();
            magnetometer_measurements.set_col(&mag_local, n).unwrap();
            //mag_local.print().unwrap();

        }

        print!("Raw gyrometer measurements:\n\n");

        for n in 0..niter {
            print!("ω[{:}Δt]\t", n);
        }

        print!("\n\n");

        gyrometer_measurements.print().unwrap();

        print!("Raw accelerometer measurements:\n\n");

        for n in 0..niter {
            print!("ω[{:}Δt]\t", n);
        }

        print!("\n\n");

        accelerometer_measurements.print().unwrap();

        print!("Raw magnetometer measurements:\n\n");

        for n in 0..niter {
            print!("ω[{:}Δt]\t", n);
        }

        print!("\n\n");

        magnetometer_measurements.print().unwrap();










        let mut ar_filter = AR::new().unwrap();

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
            3
        ).unwrap();

        let mut ar_filter_estimated_attitude = Matrix::new();   // Create a new matrix to store estimated attitude from the AR filter.
        ar_filter_estimated_attitude.init(4, niter).unwrap();   // Initialize it.
        ar_filter_estimated_attitude.fill(0.0).unwrap();        // Fill it with zero value.

        for n in 0..niter {
            let gx = gyrometer_measurements.get_element(0, n).unwrap();
            let gy = gyrometer_measurements.get_element(1, n).unwrap();
            let gz = gyrometer_measurements.get_element(2, n).unwrap();

            ar_filter.update(gx, gy, gz).unwrap();
            ar_filter_estimated_attitude.set_col(&ar_filter.get_attitude().unwrap(), n).unwrap();
            //ar_filter.print_attitude().unwrap();
        }

        print!("Estimated attitudes (AR filter):\n\n");

        for n in 1..niter+1 {
            print!("q[{:}Δt]\t", n);
        }

        print!("\n\n");

        ar_filter_estimated_attitude.print().unwrap();













        let mut aqua_filter = AQUA::new().unwrap();

        let default_gyrometer_config = GyrometerConfig::default();
        let default_accelerometer_config = AccelerometerConfig::default();
        let default_magnetometer_config = MagnetometerConfig::default();

        // Add some code here.
        aqua_filter.init(
            Mode::AM,
            //qw, qx, qy, qz,               // Initial orientation.
            None, None, None, None,         // No initial orientation.
            default_gyrometer_config,       // Gyrometer configuration.
            default_accelerometer_config,   // Accelerometer configuration.
            default_magnetometer_config,    // Magnetometer configuration.
            ts,                             // Sampling perdiod.
            false,                          // Disable adaptive.
            0.01,
            0.01,
            0.1,
            0.2,
            0.9,
            0.9
        ).unwrap();

        let mut aqua_filter_estimated_attitude = Matrix::new(); // Create a new matrix to store estimated attitude from the AR filter.
        aqua_filter_estimated_attitude.init(4, niter).unwrap(); // Initialize it.
        aqua_filter_estimated_attitude.fill(0.0).unwrap();      // Fill it with zero value.

        for n in 0..niter {
            let gx = gyrometer_measurements.get_element(0, n).unwrap();
            let gy = gyrometer_measurements.get_element(1, n).unwrap();
            let gz = gyrometer_measurements.get_element(2, n).unwrap();

            let ax = accelerometer_measurements.get_element(0, n).unwrap();
            let ay = accelerometer_measurements.get_element(1, n).unwrap();
            let az = accelerometer_measurements.get_element(2, n).unwrap();

            let mx = magnetometer_measurements.get_element(0, n).unwrap();
            let my = magnetometer_measurements.get_element(1, n).unwrap();
            let mz = magnetometer_measurements.get_element(2, n).unwrap();

            //aqua_filter.update(None, None, None, ax, ay, az, mx, my, mz).unwrap();
            aqua_filter.update(Some(gx), Some(gy), Some(gz), ax, ay, az, mx, my, mz).unwrap();
            aqua_filter_estimated_attitude.set_col(&aqua_filter.get_attitude().unwrap(), n).unwrap();
            //aqua_filter.print_attitude().unwrap();
        }

        print!("Estimated attitudes (AQUA filter):\n\n");

        for n in 1..niter+1 {
            print!("q[{:}Δt]\t", n);
        }

        print!("\n\n");

        aqua_filter_estimated_attitude.print().unwrap();
    }
}
