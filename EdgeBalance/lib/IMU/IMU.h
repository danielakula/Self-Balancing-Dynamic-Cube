#pragma once
#include <Arduino.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>

class IMU_Sensor {
public:
    IMU_Sensor(int8_t cs, SPIClass* spi_bus);
    
    bool init();
    void calibrate();
    void update();
    
    // --- LQR State Getters ---
    float getPitch();       
    float getPitchRate();   

    // --- NEW: Dashboard Telemetry Getters ---
    float getAccelX(); float getAccelY(); float getAccelZ();
    float getGyroX();  float getGyroY();  float getGyroZ();

    // --- NEW: Dashboard Tuning Setters ---
    void setAlpha(float alpha);
    void setAccelTolerance(float tol);

private:
    Adafruit_ISM330DHCX ism;
    
    int8_t _cs;
    SPIClass* _spi; 

    // --- TUNING CONSTANTS ---
    float BASE_ALPHA = 0.98f;        
    const float GRAVITY = 9.80665f;        
    float ACCEL_TOLERANCE = 1.0f;     
    
    const float PITCH_EQUILIBRIUM_TRIM = (-45.0f)*DEG_TO_RAD; 

    // Calibration Biases
    float gx_bias, gy_bias, gz_bias;
    float ax_bias, ay_bias;

    // LQR State Variables
    float current_pitch;
    float current_pitch_rate;
    bool is_seeded; 

    // --- NEW: Storage for Telemetry ---
    float ax, ay, az;
    float gx, gy, gz;

    // Timing
    unsigned long lastFilterTime;
};