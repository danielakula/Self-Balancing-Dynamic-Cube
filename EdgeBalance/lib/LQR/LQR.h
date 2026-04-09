#pragma once
#include <Arduino.h>
#include "Config.h"

class LQR {
public:
    // Constructor
    LQR();

    // The core calculation function
    float compute(float theta, float theta_dot, float wheel_vel, float dt);

    // Allows you to dynamically update gains
    void setGains(float k1, float k2, float k3, float k4);
    
    // Safety limit for the output torque/current
    void setCurrentLimit(float max_current); 

private:
    // The LQR Gain Matrix
    float K[4];

    // Augmented State Variable (Filtered Wheel Velocity)
    float filtered_wheel_vel;
    
    // LPF Tuning
    float lpf_alpha; 

    // Safety limit
    float current_limit;
};