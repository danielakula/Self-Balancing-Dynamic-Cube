#include "LQR.h"

LQR::LQR() {
K[0] = -83.2105f * 0.5f;  // Cube Angle (Theta)
K[1] = -7.2569f * 0.5f;  // Cube Velocity (Theta_dot)
K[2] = -0.1208f * 0.5f;  // Wheel Velocity
K[3] = -0.1697f * 0.5f;  // Filtered Wheel Velocity (Augmented state)

    filtered_wheel_vel = 0.0f;

    lpf_alpha = 0.0435f; 
    
    current_limit = MotorTuning.current_limit; 
}

void LQR::setGains(float k1, float k2, float k3, float k4) {
    K[0] = k1;
    K[1] = k2;
    K[2] = k3;
    K[3] = k4;
}

void LQR::setCurrentLimit(float max_current) { 
    current_limit = max_current;
}

float LQR::compute(float theta, float theta_dot, float wheel_vel, float dt) {
    // Augmented State
    filtered_wheel_vel = (lpf_alpha * wheel_vel) + ((1.0f - lpf_alpha) * filtered_wheel_vel);

    // Base LQR Control Law
    float u = -(K[0] * theta + K[1] * theta_dot + K[2] * wheel_vel + K[3] * filtered_wheel_vel);

    // ANTI-STICTION (FRICTION COMPENSATION)
    const float STATIC_FRICTION = 0.15f; 
    
    if (u > 0.01f) {
        u += STATIC_FRICTION;
    } else if (u < -0.01f) {
        u -= STATIC_FRICTION;
    }

    // Saturation
    if (u > current_limit) u = current_limit;
    if (u < -current_limit) u = -current_limit;

    return u;
}