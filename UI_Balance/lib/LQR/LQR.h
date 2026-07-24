#pragma once
#include <Arduino.h>
#include "Config.h"

class LQR {
public:
    LQR();
    float compute(float theta, float theta_dot, float wheel_vel, float dt);
    void setGains(float k1, float k2, float k3);

private:
    float K[4]; 
    float current_limit;
    float friction_comp;
};