#pragma once
#include <Arduino.h>
#include "Config.h"

class LQR {
public:
    LQR();

    // The core calculation function
    float compute(float theta, float theta_dot, float wheel_vel, float dt);

    // Dynamic updates via Web Dashboard
    void setGains(float k1, float k2, float k3);
    
    void setCurrentLimit(float max_current);
    void setFrictionComp(float comp);

private:
    // The LQR Gain Matrix (Size 4 so we can use 1-based indexing: K[1], K[2], K[3])
    float K[4]; 

    float current_limit;
    float friction_comp;
};