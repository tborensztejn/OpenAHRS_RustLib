extern crate quaternion;
extern crate linalg;
extern crate utils;
extern crate libm;

use crate::gyroscope::Gyroscope;
use crate::accelerometer::Accelerometer;
use crate::magnetometer::Magnetometer;
use quaternion::quaternion::Quaternion;
use crate::common::OpenAHRSError;

/*
pub const MARG: u8 = 1; // This mode uses the gyroscope, the accelerometer and the magnetometer.
pub const AM:   u8 = 2; // This mode uses only the accelerometer and the magnetometer.
*/

#[derive(Debug)]
#[derive(PartialEq)]
pub enum Mode {
    MARG = 1,
    AM = 2,
}

/*
impl Mode {
    fn is_valid_mode(value: u8) -> Option<Mode> {
        match value {
            1 => Some(Mode::MARG),
            2 => Some(Mode::AM),
            _ => None,
        }
    }
}
*/

#[derive(Debug)]
pub struct AQUA {
    gyr: Gyroscope,
    acc: Accelerometer,
    mag: Magnetometer,

    orientation: Quaternion,

    ts: f32,
    adaptive: bool,
    alpha: f32,
    beta: f32,
    t1: f32,
    t2: f32,
    t_acc: f32,
    t_mag: f32,
    mode: Mode,
    order: u8,
    initialized: bool,
}

impl AQUA {
    pub fn new() -> Result<Self, OpenAHRSError> {
        let aqua = Self {
            // Filter sensors.
            gyr: Gyroscope::new()?,
            acc: Accelerometer::new()?,
            mag: Magnetometer::new()?,

            orientation: Quaternion::new()?,    // Estimated attitude by the filter.

            // Filter settings.
            ts:         0.01_f32,               // Sampling period.
            adaptive:   true,                   // Activate or not the adaptive gain.
            alpha:      0.01_f32,               // ...
            beta:       0.01_f32,               // ...
            t1:         0.1_f32,                // Adaptative gain first treshold.
            t2:         0.2_f32,                // Adaptative gain second treshold.
            t_acc:      0.9_f32,                // Interpolation treshold for the partial orientation (quaternion) determined from accelerometer measurements.
            t_mag:      0.9_f32,                // Interpolation treshold for the partial orientation (quaternion) determined from magnetometer measurements.
            //mode:       MARG,                   // Mode of the filter.
            mode:       Mode::MARG,             // Mode of the filter.
            order:      2_u8,                   // Order of the numerical integration method.
            //method:     Method::TAYLOR_SERIES,  // Numerical integration method.

            initialized: false,             // Initialization status.
        };

        Ok(aqua)    // Return the structure with no error.
    }

    pub fn init(self: &mut Self,
        // Mode of the filter.
        //mode: u8,
        mode: Mode,

        // Initial attitude (optionnal).
        qw: Option<f32>, qx: Option<f32>, qy: Option<f32>, qz: Option<f32>,

        // Gyroscope settings.
        x_axis_scaling_correction_factor: Option<f32>, y_axis_scaling_correction_factor: Option<f32>, z_axis_scaling_correction_factor: Option<f32>,
        xy_axes_non_orthogonality_correction_factor: Option<f32>, xz_axes_non_orthogonality_correction_factor: Option<f32>,
        yx_axes_non_orthogonality_correction_factor: Option<f32>, yz_axes_non_orthogonality_correction_factor: Option<f32>,
        zx_axes_non_orthogonality_correction_factor: Option<f32>, zy_axes_non_orthogonality_correction_factor: Option<f32>,
        x_axis_static_bias: Option<f32>, y_axis_static_bias: Option<f32>, z_axis_static_bias: Option<f32>,

    ) -> Result<(), OpenAHRSError> {
        /*
        let valid_mode = match Mode::is_valid_mode(mode) {
            Some(valid_mode) => valid_mode,
            None => return Err(LinalgError::InvalidAQUAMode),
        };

        match valid_mode {
            Mode::MARG => {
                // Initialize the gyroscope.
                // Initialize the accelerometer.
                // Initialize the magnetometer.
            }

            Mode::AM => {
                // Initialize the accelerometer.
                // Initialize the magnetometer.
            }
        }
        */

        if mode == Mode::MARG {
            /* Initialize the gyroscope */

            // Axes scaling factors.
            let x_axis_scaling_correction_factor = x_axis_scaling_correction_factor.unwrap_or(1.0_f32);
            let y_axis_scaling_correction_factor = y_axis_scaling_correction_factor.unwrap_or(1.0_f32);
            let z_axis_scaling_correction_factor = z_axis_scaling_correction_factor.unwrap_or(1.0_f32);

            // Axes non-orthogonality correction factors.
            let xy_axes_non_orthogonality_correction_factor = xy_axes_non_orthogonality_correction_factor.unwrap_or(0.0_f32);
            let xz_axes_non_orthogonality_correction_factor = xz_axes_non_orthogonality_correction_factor.unwrap_or(0.0_f32);
            let yx_axes_non_orthogonality_correction_factor = yx_axes_non_orthogonality_correction_factor.unwrap_or(0.0_f32);
            let yz_axes_non_orthogonality_correction_factor = yz_axes_non_orthogonality_correction_factor.unwrap_or(0.0_f32);
            let zx_axes_non_orthogonality_correction_factor = zx_axes_non_orthogonality_correction_factor.unwrap_or(0.0_f32);
            let zy_axes_non_orthogonality_correction_factor = zy_axes_non_orthogonality_correction_factor.unwrap_or(0.0_f32);

            // Axes static biases correction factors.
            let x_axis_static_bias = x_axis_static_bias.unwrap_or(0.0_f32);
            let y_axis_static_bias = y_axis_static_bias.unwrap_or(0.0_f32);
            let z_axis_static_bias = z_axis_static_bias.unwrap_or(0.0_f32);

            self.gyr.init(
                x_axis_scaling_correction_factor, y_axis_scaling_correction_factor, z_axis_scaling_correction_factor,
                xy_axes_non_orthogonality_correction_factor, xz_axes_non_orthogonality_correction_factor,
                yx_axes_non_orthogonality_correction_factor, yz_axes_non_orthogonality_correction_factor,
                zx_axes_non_orthogonality_correction_factor, zy_axes_non_orthogonality_correction_factor,
                x_axis_static_bias, y_axis_static_bias, z_axis_static_bias
            )?;

            /* Initialize the accelerometer. */
            // Add some code here.

            /* Initialize the magnetometer. */
            // Add some code here.
        } else if mode == Mode::AM {
            /* Initialize the accelerometer. */
            // Add some code here.

            /* Initialize the magnetometer. */
            // Add some code here.
        } else {
            return Err(OpenAHRSError::InvalidAQUAMode);
        }

        // Check whether all quaternion coordinates are present or absent.
        let quat = match (qw, qx, qy, qz) {
            (Some(qw), Some(qx), Some(qy), Some(qz)) => Some((qw, qx, qy, qz)),
            (None, None, None, None) => None,
            _ => return Err(OpenAHRSError::InvalidQuaternion),
        };

        // Set initial orientation manually.
        if let Some((qw, qx, qy, qz)) = quat {
            self.orientation.fill(qw, qx, qy, qz)?;
        } else {    // Automatically initialize orientation.
            // Add some code here.
        }

        Ok(())
    }

    pub fn update(self: &mut Self,
        gx: Option<f32>, gy: Option<f32>, gz: Option<f32>,  // Gyroscope raw measurements.
        ax: f32, ay: f32, az: f32,                          // Accelerometer raw measurements.
        mx: f32, my: f32, mz: f32                           // Magnetometer raw measurements.
    ) -> Result<(), OpenAHRSError> {
        // Add some code here.

        

        Ok(())  // Return no error.
    }
}
