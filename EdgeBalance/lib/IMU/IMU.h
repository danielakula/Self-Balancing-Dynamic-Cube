#pragma once
#include <Arduino.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>

class IMU_Sensor {
public:
    IMU_Sensor(int8_t cs, SPIClass* spi_bus);
    
    // Core functions
    bool init();
    void calibrate();
    void update();
    
    // Direct getters for your LQR State Matrix
    float getPitch();       
    float getPitchRate();   

private:
    Adafruit_ISM330DHCX ism;
    
    // Hardware Pins & Bus
    int8_t _cs;
    SPIClass* _spi; // Pointer to the shared SPI bus

    // --- TUNING CONSTANTS ---
    const float BASE_ALPHA = 0.998f;        
    const float GRAVITY = 9.80665f;         
    const float ACCEL_TOLERANCE = 1.0f;     
    
    const float PITCH_EQUILIBRIUM_TRIM = (-45.0f-1.0f)*DEG_TO_RAD; 

    // Calibration Biases
    float gx_bias, gy_bias, gz_bias;
    float ax_bias, ay_bias;

    // LQR State Variables
    float current_pitch;
    float current_pitch_rate;
    bool is_seeded;

    // Timing
    unsigned long lastFilterTime;
};