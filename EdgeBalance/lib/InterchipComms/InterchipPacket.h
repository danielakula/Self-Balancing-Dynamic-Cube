#pragma once
#include <Arduino.h>

struct InterchipPacket {
    float target1; // current targets
    float target2;
    float target3;
    float targetPitch; // target equilibrium point

    // Controllers
    float Kp_outer;
    float Ki_outer;
    float k1;
    float k2;
    float k3;

    // IMU / Filter Tuning
    float BASE_ALPHA;
    float ACCEL_TOLERANCE;

    // State Machine
    uint8_t robotState;  // 0=disabled, 1=edge, 2=corner
    uint8_t targetEdge;  // 1, 2, or 3 (ignored if corner balancing)

    uint32_t faultCode;  

    // Motor States
    float motor1Velocity; float motor2Velocity; float motor3Velocity; // Wheel velocities
    float motor1Current;  float motor2Current;  float motor3Current; // Motor measured currents
    
    // IMU / Kinematics
    float ax; float ay; float az; 
    float gx; float gy; float gz;
    float pitch;
    float pitchRate;

    // Auto-sync
    float active_Kp_outer;
    float active_Ki_outer;
    float active_k1;
    float active_k2;
    float active_k3;
} __attribute__((packed));