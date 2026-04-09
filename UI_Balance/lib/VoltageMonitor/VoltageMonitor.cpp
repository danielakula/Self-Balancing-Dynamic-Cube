#include "VoltageMonitor.h"

VoltageMonitor::VoltageMonitor(int pin, float dividerRatio, float calibrationTrim)
    : _pin(pin), _dividerRatio(dividerRatio), _calibrationTrim(calibrationTrim) {}

void VoltageMonitor::begin() {
    analogRead(_pin); 
    delay(10); 

    analogReadResolution(12);
    analogSetPinAttenuation(_pin, ADC_11db);
}

float VoltageMonitor::readVoltage() {
    uint32_t totalMilliVolts = 0;
    
    for (int i = 0; i < 64; i++) {
        totalMilliVolts += analogReadMilliVolts(_pin);
    }
    
    float avgMilliVolts = (float)totalMilliVolts / 64.0f;
    
    return (avgMilliVolts / 1000.0f) * _dividerRatio * _calibrationTrim;
}

bool VoltageMonitor::isUnderVoltage(float minVoltage) {
    return readVoltage() < minVoltage;
}

bool VoltageMonitor::isOverVoltage(float maxVoltage) {
    return readVoltage() > maxVoltage;
}