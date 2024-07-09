extern crate quaternion;
extern crate linalg;
extern crate utils;
extern crate libm;

use crate::gyrometer::{GyrometerConfig, Gyrometer};
use crate::accelerometer::{AccelerometerConfig, Accelerometer};
use crate::magnetometer::{MagnetometerConfig, Magnetometer};
use crate::common::OpenAHRSError;

use quaternion::quaternion::Quaternion;
use linalg::matrix::Matrix;
use linalg::vector::Vector;
use linalg::common::EPSILON;
//use libm::{sqrtf, fabsf};

#[derive(Debug)]
pub struct Davenport {
    // Filter sensors (accelerometer and magnetometer).
    acc: Accelerometer,
    mag: Magnetometer,

    attitude: Vector<f32>,  // Estimated attitude by the filter.

    ts: f32,                // Sampling period.
    w1: f32,                // ...
    w2: f32,                // ...
    g: f32,                 // Earth gravitational acceleration magnitude.
    mdip: f32,              // Earth magnetic field dip.

    initialized: bool,      // Initialization status.
}

impl Davenport {
    /// This method is used to create a new Davenport filter.
    pub fn new() -> Result<Self, OpenAHRSError> {
        let davenport = Self {
            // Filter sensors (accelerometer and magnetometer).
            acc: Accelerometer::new()?,
            mag: Magnetometer::new()?,

            attitude: Vector::new(),        // Estimated attitude by the filter as a quaternion.

            // Filter settings.
            ts:         0.01,               // Default sampling period.
            w1:         0.1,                // Default ...
            w2:         0.2,                // Default ...
            g:          9.81,               // Default Earth gravitational acceleration magnitude.
            mdip:       48.0,               // Default Earth magnetic field dip.

            initialized: false,             // Default initialization status.
        };

        Ok(davenport)   // Return the new filter with no error.
    }
}
