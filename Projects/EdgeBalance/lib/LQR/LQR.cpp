#include "LQR.h"

LQR::LQR() {
    // 1. PLACE YOUR MATLAB K-MATRIX GAINS HERE!
K[0] = -83.2105f * 0.5f;  // Cube Angle (Theta)
K[1] = -7.2569f * 0.5f;  // Cube Velocity (Theta_dot)
K[2] = -0.1208f * 0.5f;  // Wheel Velocity
K[3] = -0.1697f * 0.5f;  // Filtered Wheel Velocity (Augmented state)

    filtered_wheel_vel = 0.0f;
    
    // FIXED: Dropped alpha to 5% so it actually acts as a heavy low-pass filter to find the COM
    lpf_alpha = 0.0435f; 
    
    // FIXED: Safe default. Update this from main.cpp using setCurrentLimit()
    current_limit = MotorTuning.current_limit; 
}

void LQR::setGains(float k1, float k2, float k3, float k4) {
    K[0] = k1;
    K[1] = k2;
    K[2] = k3;
    K[3] = k4;
}

void LQR::setCurrentLimit(float max_current) { // FIXED: Variable names match header
    current_limit = max_current;
}

float LQR::compute(float theta, float theta_dot, float wheel_vel, float dt) {
    // 1. Augmented State
    filtered_wheel_vel = (lpf_alpha * wheel_vel) + ((1.0f - lpf_alpha) * filtered_wheel_vel);

    // 2. Base LQR Control Law
    float u = -(K[0] * theta + K[1] * theta_dot + K[2] * wheel_vel + K[3] * filtered_wheel_vel);

    // 3. ANTI-STICTION (FRICTION COMPENSATION)
    // Find the minimum Amps required to make the wheel *just barely* move in the air.
    // E.g., if it takes 0.15A to overcome cogging, set this to 0.15f.
    const float STATIC_FRICTION = 0.15f; 
    
    // Only apply if the motor is actually commanded to do something
    if (u > 0.01f) {
        u += STATIC_FRICTION;
    } else if (u < -0.01f) {
        u -= STATIC_FRICTION;
    }

    // 4. Saturation
    if (u > current_limit) u = current_limit;
    if (u < -current_limit) u = -current_limit;

    return u;
}