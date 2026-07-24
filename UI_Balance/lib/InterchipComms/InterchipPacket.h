#pragma once
#include <Arduino.h>

struct InterchipPacket {

    uint8_t packetType; // 0 = Command (Slave->Master), 1 = Telemetry (Master->Slave)
    
    // COMMANDS (Wi-Fi MCU -> Control MCU) 
    float target1; float target2; float target3;
    float targetPitch; 

    float Kp_outer; float Ki_outer;
    float k1; float k2; float k3;
    float BASE_ALPHA; float ACCEL_TOLERANCE;

    uint8_t robotState;  
    uint8_t targetEdge;  

    // TELEMETRY (Control MCU -> Wi-Fi MCU) 
    uint32_t faultCode;  
    float motor1Velocity; float motor2Velocity; float motor3Velocity;
    float motor1Current;  float motor2Current;  float motor3Current;
    
    float ax; float ay; float az;
    float gx; float gy; float gz;
    float pitch; float pitchRate;
    float q0, q1, q2, q3;

    // Active Parameters for Dashboard Auto-Sync
    float active_Kp_outer; float active_Ki_outer;
    float active_k1; float active_k2; float active_k3;
} __attribute__((packed));