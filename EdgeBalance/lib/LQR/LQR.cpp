#include "LQR.h"

LQR::LQR() {
    // 1. Initialize using the Single Source of Truth from Config.h
    K[1] = DEFAULT_LQR_K1;  // Cube Angle (Theta)
    K[2] = DEFAULT_LQR_K2;  // Cube Velocity (Theta_dot)
    K[3] = DEFAULT_LQR_K3;  // Wheel Velocity
    
    current_limit = MotorTuning.current_limit; 
    friction_comp = MotorTuning.friction_comp;
}

void LQR::setFrictionComp(float comp) {
    friction_comp = comp;
}

void LQR::setGains(float k1, float k2, float k3) {
    K[1] = k1;
    K[2] = k2;
    K[3] = k3;
}

void LQR::setCurrentLimit(float max_current) { 
    current_limit = max_current;
}

float LQR::compute(float theta, float theta_dot, float wheel_vel, float dt) {

    float u = -(K[1] * theta + K[2] * theta_dot + K[3] * wheel_vel); // + K[4] * wheel_integral;

    // Friction Compensation
    if (u > 0.001f) {
        u += friction_comp;
    } else if (u < -0.001f) {
        u -= friction_comp;
    } else {
        u = 0.0f; // Absolute dead center
    }

    // 4. Output Saturation
    if (u > current_limit) u = current_limit;
    if (u < -current_limit) u = -current_limit;

    return u;
}