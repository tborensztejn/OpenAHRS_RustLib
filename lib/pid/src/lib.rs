
#[derive(Debug)]
pub struct PID {
    kp: f64,            // Gain of the proportional action.
    ki: f64,            // Gain of the integral action.
    kd: f64,            // Gain of the derivative action.
    ts: f64,            // Sampling period.
    fc: f64,            // Low-pass filter cutoff frequency of the derivative action.
    tau: f64,           // Low-pass filter time constant.
    sat_min: f64,       // Minimum saturator output value.
    sat_max: f64,       // Maximum saturator output value.
    initialized: bool,  // Initialization flag.
}

impl PID {
    // This function is used to create a new PID controller.
    pub fn new() -> Self {
        Self {
            kp: 0.0_f64,
            ki: 0.0_f64,
            kd: 0.0_f64,
            ts: 0.1_f64,
        
