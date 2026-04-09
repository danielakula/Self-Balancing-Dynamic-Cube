#pragma once
#include <Arduino.h>

class VoltageMonitor {
public:
    VoltageMonitor(int pin, float dividerRatio, float calibrationTrim);
    
    void begin();
    float readVoltage();
    bool isUnderVoltage(float minVoltage);
    bool isOverVoltage(float maxVoltage);

private:
    int _pin;
    float _dividerRatio;
    float _calibrationTrim;
};