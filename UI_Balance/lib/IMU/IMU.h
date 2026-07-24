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
    
    float getPitch();       
    float getPitchRate();   
    float getAccelX(); float getAccelY(); float getAccelZ();
    float getGyroX();  float getGyroY();  float getGyroZ();

    void setAlpha(float alpha);
    void setAccelTolerance(float tol);

private:
    Adafruit_ISM330DHCX ism;
    int8_t _cs; SPIClass* _spi; 
    
    float BASE_ALPHA = 0.98f;        
    const float GRAVITY = 9.80665f;        
    float ACCEL_TOLERANCE = 1.0f;     
    const float PITCH_EQUILIBRIUM_TRIM = (-45.0f)*DEG_TO_RAD; 

    float gx_bias, gy_bias, gz_bias, ax_bias, ay_bias;
    float current_pitch, current_pitch_rate;
    bool is_seeded; 
    float ax, ay, az, gx, gy, gz;
    unsigned long lastFilterTime;
};