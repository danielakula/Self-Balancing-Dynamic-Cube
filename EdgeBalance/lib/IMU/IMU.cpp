#include "IMU.h"
#include <cmath>

IMU_Sensor::IMU_Sensor(int8_t cs, SPIClass* spi_bus) :
    _cs(cs), _spi(spi_bus),
    gx_bias(0.0f), gy_bias(0.0f), gz_bias(0.0f),
    ax_bias(0.0f), ay_bias(0.0f),
    current_pitch(0.0f), current_pitch_rate(0.0f),
    is_seeded(false), lastFilterTime(0) {}

bool IMU_Sensor::init() {
    if (!ism.begin_SPI(_cs, _spi)) {
        return false;
    }

    ism.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    ism.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    ism.setAccelDataRate(LSM6DS_RATE_416_HZ);
    ism.setGyroDataRate(LSM6DS_RATE_416_HZ);

    lastFilterTime = micros();
    return true;
}

void IMU_Sensor::calibrate() {
    Serial.println("KEEP STILL - CALIBRATING IMU...");
    delay(500); 

    const int CALIB_SAMPLES = 1000;
    double sumGX = 0, sumGY = 0, sumGZ = 0;
    int count = 0;
    unsigned long calibLast = micros();

    while (count < CALIB_SAMPLES) {
        unsigned long now = micros();
        if ((now - calibLast) >= 2000) { 
            calibLast = now;
            sensors_event_t accel, gyro, temp;
            ism.getEvent(&accel, &gyro, &temp);

            sumGX += gyro.gyro.x; 
            sumGY += gyro.gyro.y; 
            sumGZ += gyro.gyro.z;
            count++;
        }
    }

    gx_bias = sumGX / CALIB_SAMPLES;
    gy_bias = sumGY / CALIB_SAMPLES;
    gz_bias = sumGZ / CALIB_SAMPLES;

    Serial.println("IMU Calibration done!");
}

void IMU_Sensor::update() {
    unsigned long now = micros();
    float dt = (now - lastFilterTime) / 1000000.0f;
    
    if (dt > 0.05f) dt = 0.002f; 
    
    lastFilterTime = now;

    sensors_event_t accel, gyro, temp;
    ism.getEvent(&accel, &gyro, &temp);

    // Fetch axes
    float gy = gyro.gyro.y - gy_bias;
    float gz = gyro.gyro.z - gz_bias; 
    
    float ax = accel.acceleration.x - ax_bias;
    float ay = accel.acceleration.y - ay_bias; 
    float az = accel.acceleration.z; 

    if (abs(gy) < 0.001f) gy = 0.0f;

    current_pitch_rate = gy;

    float accel_pitch_rad = atan2(-ax, sqrt(ay * ay + az * az));

    if (!is_seeded) {
        current_pitch = accel_pitch_rad;
        is_seeded = true;
    }

    // Adaptive Gain Logic with Yaw Lockout
    float accel_mag = sqrt(ax * ax + ay * ay + az * az);
    float alpha = BASE_ALPHA;

    // If linear acceleration is violent OR if the robot is yawing/twisting heavily (> 0.5 rad/s)
    if (abs(accel_mag - GRAVITY) > ACCEL_TOLERANCE || abs(gz) > 0.5f) {
        // 100% trust in Gyro
        alpha = 1.0f; 
    }

    // Complementary Filter Math
    current_pitch = alpha * (current_pitch + gy * dt) + (1.0f - alpha) * accel_pitch_rad;
}

float IMU_Sensor::getPitch() {
    return current_pitch - PITCH_EQUILIBRIUM_TRIM;
}

float IMU_Sensor::getPitchRate() {
    return current_pitch_rate;
}