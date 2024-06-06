#![cfg_attr(not(feature = "std"), no_std)]

pub mod quaternion;

#[cfg(test)]
mod quaternion_tests {
    use crate::quaternion::Quaternion;

    // This test function is used to check that there are no errors when creating a quaternion. No error expected.
    #[test]
    //#[ignore]
    fn init_test_1() {
        let result = Quaternion::new();

        assert!(result.is_ok(), "Unexpected result: {:?}.", result);
    }

    // This test function is used to check ...
    #[test]
    fn fill_test() {
        let mut quat: Quaternion = Quaternion::new().unwrap();  // Create a new quaternion.

        // Define quaternion elements.
        let qw: f32 = 1.0;
        let qx: f32 = 2.0;
        let qy: f32 = 3.0;
        let qz: f32 = 4.0;

        let result = quat.fill(qw, qx, qy, qz); // Set quaternion elements.
        assert!(result.is_ok(), "Unexpected result: {:?}.", result);

        let mut q = quat.get_qw();
        assert!(q.is_ok(), "Unexpected result: {:?}.", q);
        assert_eq!(q.unwrap(), qw, "Unexpected element value.");

        q = quat.get_qx();
        assert!(q.is_ok(), "Unexpected result: {:?}.", q);
        assert_eq!(q.unwrap(), qx, "Unexpected element value.");

        q = quat.get_qy();
        assert!(q.is_ok(), "Unexpected result: {:?}.", q);
        assert_eq!(q.unwrap(), qy, "Unexpected element value.");

        q = quat.get_qz();
        assert!(q.is_ok(), "Unexpected result: {:?}.", q);
        assert_eq!(q.unwrap(), qz, "Unexpected element value.");
    }

    // This test function is used to check ...
    #[test]
    fn mull_test() {
        let mut p: Quaternion = Quaternion::new().unwrap();  // Create a new quaternion.
        let mut q: Quaternion = Quaternion::new().unwrap();  // Create a new quaternion.
        let mut pq: Quaternion = Quaternion::new().unwrap();  // Create a new quaternion.

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

        let mut result = p.fill(pw, px, py, pz); // Set quaternion elements.
        assert!(result.is_ok(), "Unexpected result: {:?}.", result);
        result = q.fill(qw, qx, qy, qz); // Set quaternion elements.
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
