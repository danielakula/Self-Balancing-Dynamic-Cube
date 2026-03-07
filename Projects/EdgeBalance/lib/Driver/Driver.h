#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>
#include <atomic>
#include "Config.h" // Your global structs

class Driver {
public:
    // The Constructor
    Driver(const MotorPins& pins, const MotorConfig& config);

    // Public Methods
    void begin(SPIClass* spiBus); 
    void runFOC();             
    void setTarget(float targetCurrent);
    void emergencyStop();
    void enable();
    
    // Fault Methods (Added missing declarations!)
    bool hasHardwareFault(); 
    void printDetailedFaults(const char* motorName);
    
    // Telemetry Getters
    float getVelocity();
    float getCurrentQ();

private:
    // Internal Variables
    MotorPins _pins;
    MotorConfig _config;
    
    // Thread-safe flags
    std::atomic<float> _target;
    std::atomic<bool> _enabled;

    // Simple FOC Objects
    BLDCDriver6PWM _driver;
    BLDCMotor _motor;
    MagneticSensorSPI _sensor;
    LowsideCurrentSense _currentSense;

    // Internal DRV8323 Helpers
    SPIClass* _spi; 
    void drvWriteSpi(uint8_t address, uint16_t data);
    uint16_t drvReadSpi(uint8_t address);
    void initDRV8323S();
};