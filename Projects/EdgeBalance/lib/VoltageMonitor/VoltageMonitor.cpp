#include "VoltageMonitor.h"

VoltageMonitor::VoltageMonitor(int pin, float dividerRatio, float calibrationTrim)
    : _pin(pin), _dividerRatio(dividerRatio), _calibrationTrim(calibrationTrim) {}

void VoltageMonitor::begin() {
    // 1. DUMMY READ: Forces the ESP32 Core to initialize the ADC hardware on this pin
    analogRead(_pin); 
    delay(10); // Give the silicon a tiny moment to settle

    // 2. Now it is safe to set the resolution and attenuation
    analogReadResolution(12);
    analogSetPinAttenuation(_pin, ADC_11db);
}

float VoltageMonitor::readVoltage() {
    uint32_t totalMilliVolts = 0;
    
    // Burst read 64 samples. No delay! 
    // This takes ~0.6 milliseconds, which is totally safe for Core 0.
    for (int i = 0; i < 64; i++) {
        totalMilliVolts += analogReadMilliVolts(_pin);
    }
    
    float avgMilliVolts = (float)totalMilliVolts / 64.0f;
    
    // Apply your divider and trim math
    return (avgMilliVolts / 1000.0f) * _dividerRatio * _calibrationTrim;
}

bool VoltageMonitor::isUnderVoltage(float minVoltage) {
    return readVoltage() < minVoltage;
}

bool VoltageMonitor::isOverVoltage(float maxVoltage) {
    return readVoltage() > maxVoltage;
}