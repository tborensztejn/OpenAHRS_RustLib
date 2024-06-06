
#[derive(Debug)]
pub struct PID {
    kp: f32,            // Gain of the proportional action.
    ki: f32,            // Gain of the integral action.
    kd: f32,            // Gain of the derivative action.
    ts: f32,            // Sampling period.
    fc: f32,            // Low-pass filter cutoff frequency of the derivative action.
    tau: f32,           // Low-pass filter time constant.
    sat_min: f32,       // Minimum saturator output value.
    sat_max: f32,       // Maximum saturator output value.
    initialized: bool,  // Initialization flag.
}

impl PID {
    // This function is used to create a new PID controller.
    pub fn new() -> Self {
        Self {
            kp: 0.0_f32,
            ki: 0.0_f32,
            kd: 0.0_f32,
            ts: 0.1_f32,
        }
    }
}
