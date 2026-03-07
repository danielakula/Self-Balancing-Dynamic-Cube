#pragma once
#include <Arduino.h>

struct InterchipPacket {
    float targetCurrent; // Sent by Master -> Slave
    uint32_t faultCode;  // Shared between both
    
    // Telemetry (Sent by Slave -> Master)
    float motor2Velocity;
    float motor3Velocity;
    float motor2Current;
    float motor3Current;
} __attribute__((packed));