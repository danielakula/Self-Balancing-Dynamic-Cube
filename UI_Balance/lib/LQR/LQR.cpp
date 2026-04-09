#include "LQR.h"

LQR::LQR() {
    K[1] = DEFAULT_LQR_K1; K[2] = DEFAULT_LQR_K2; K[3] = DEFAULT_LQR_K3;
}

void LQR::setGains(float k1, float k2, float k3) { K[1] = k1; K[2] = k2; K[3] = k3; }

float LQR::compute(float theta, float theta_dot, float wheel_vel, float dt) {

    float u = -(K[1] * theta + K[2] * theta_dot + K[3] * wheel_vel); 

    if (u > 0.001f) u += MotorTuning.friction_comp;
    else if (u < -0.001f) u -= MotorTuning.friction_comp;
    else u = 0.0f; 

    if (u > current_limit) u = MotorTuning.current_limit;
    if (u < -current_limit) u = -MotorTuning.current_limit;

    return u;
}