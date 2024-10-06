#[derive(Default)]
pub struct PID {
    kp: f32,            // Gain of the proportional action.
    ki: f32,            // Gain of the integral action.
    kd: f32,            // Gain of the derivative action.
    ts: f32,            // Sampling period.
    /*
    fc: f32,            // Low-pass filter cutoff fequency of the derivative action.
    tau: f32,           // Low-pass filter time constant.
    sat_min: f32,       // Minimum saturator output value.
    sat_max: f32,       // Maximum saturator output value.
    initialized: bool,  // Initialization flag.
    */
}

impl PID {
    /// This method is used to create a new PID controller.
    pub fn new() -> Self {
        Self {
            ts: 0.1_f32,
            ..Self::default()
        }
    }
}
