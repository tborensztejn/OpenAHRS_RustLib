#![cfg_attr(not(feature = "std"), no_std)]

pub mod quaternion;

#[cfg(test)]
mod quaternion_tests {
    use linalg::vector::Vector;
    use crate::quaternion::Quaternion;

    // This test function is used to check ...
    #[test]
    fn fill_test() {
        let mut quat: Vector<f32> = Vector::new();  // Create a new vector that will be used as a quaternion.
        quat.init(4).unwrap();                      // Initialize it.

        // Define quaternion elements.
        let qw: f32 = 1.0;
        let qx: f32 = 2.0;
        let qy: f32 = 3.0;
        let qz: f32 = 4.0;

        let result = quat.fillq(qw, qx, qy, qz);    // Set quaternion components.
        assert!(result.is_ok(), "Unexpected result: {:?}.", result);

        let mut component = quat.get_qw();
        assert!(component.is_ok(), "Unexpected result: {:?}.", component);
        assert_eq!(component.unwrap(), qw, "Unexpected element value.");

        component = quat.get_qx();
        assert!(component.is_ok(), "Unexpected result: {:?}.", component);
        assert_eq!(component.unwrap(), qx, "Unexpected element value.");

        component = quat.get_qy();
        assert!(component.is_ok(), "Unexpected result: {:?}.", component);
        assert_eq!(component.unwrap(), qy, "Unexpected element value.");

        component = quat.get_qz();
        assert!(component.is_ok(), "Unexpected result: {:?}.", component);
        assert_eq!(component.unwrap(), qz, "Unexpected element value.");
    }

    // This test function is used to check ...
    #[test]
    fn mull_test() {
        let mut p: Vector<f32> = Vector::new();     // Create a new vector that will be used as a quaternion.
        let mut q: Vector<f32> = Vector::new();     // Create a new vector that will be used as a quaternion.
        let mut pq: Vector<f32> = Vector::new();    // Create a new vector that will be used as a quaternion.

        // Initialize them.
        p.init(4).unwrap();
        q.init(4).unwrap();
        pq.init(4).unwrap();

        // Define p quaternion elements.
        let pw: f32 = 2.0;
        let px: f32 = 3.0;
        let py: f32 = 2.0;
        let pz: f32 = 3.0;

        // Define q quaternion elements.
        let qw: f32 = 3.0;
        let qx: f32 = 2.0;
        let qy: f32 = 3.0;
        let qz: f32 = 2.0;

        let mut result = p.fillq(pw, px, py, pz);   // Set quaternion components.
        assert!(result.is_ok(), "Unexpected result: {:?}.", result);
        result = q.fillq(qw, qx, qy, qz);   // Set quaternion components.
        assert!(result.is_ok(), "Unexpected result: {:?}.", result);

        result = pq.mul(&p, &q);
        assert!(result.is_ok(), "Unexpected result: {:?}.", result);
        //pq.print().unwrap();

        let mut result = pq.get_qw();
        assert!(result.is_ok(), "Unexpected result: {:?}.", result);
        assert_eq!(result.unwrap(), -12.0_f32, "Unexpected element value.");

        result = pq.get_qx();
        assert!(result.is_ok(), "Unexpected result: {:?}.", result);
        assert_eq!(result.unwrap(), 8.0_f32, "Unexpected element value.");

        result = pq.get_qy();
        assert!(result.is_ok(), "Unexpected result: {:?}.", result);
        assert_eq!(result.unwrap(), 12.0_f32, "Unexpected element value.");

        result = pq.get_qz();
        assert!(result.is_ok(), "Unexpected result: {:?}.", result);
        assert_eq!(result.unwrap(), 18.0_f32, "Unexpected element value.");
    }
}
