#pragma once
#include <Arduino.h>
#include <PacketSerial.h>
#include <atomic>
#include "InterchipPacket.h"

class InterchipComms {
public:
    InterchipComms(HardwareSerial& serialPort, int rxPin, int txPin);

    void begin(unsigned long baudRate = 1000000);
    void update();

    // Sending Data
    void setTelemetry(float m2Vel, float m3Vel, float m2Cur, float m3Cur); // Used by Slave
    void sendPacket(float target, uint32_t faultCode); // Updated to uint32_t

    // Reading Data & Safety
    float getTargetCurrent();
    uint32_t getRemoteFaultCode(); // Updated to uint32_t
    bool isConnectionAlive(uint32_t timeoutMs = 15); 
    
    // Telemetry Getters (Used by Master)
    float getMotor2Velocity();
    float getMotor3Velocity();
    float getMotor2Current();
    float getMotor3Current();

private:
    HardwareSerial& _serial;
    PacketSerial _cobsSerial;
    int _rxPin, _txPin;

    // Thread-safe data storage
    std::atomic<float> _latestTarget;
    std::atomic<uint32_t> _remoteFaultCode;
    std::atomic<uint32_t> _lastPacketTime;
    
    // Telemetry Storage
    std::atomic<float> _m2Vel;
    std::atomic<float> _m3Vel;
    std::atomic<float> _m2Cur;
    std::atomic<float> _m3Cur;

    static void onPacketReceived(const uint8_t* buffer, size_t size);
};