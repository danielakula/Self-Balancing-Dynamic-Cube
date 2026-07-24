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

    // Setters for Master -> Slave (Telemetry)
    void setTelemetryMotors(float v1, float v2, float v3, float c1, float c2, float c3);
    void setTelemetryIMU(float ax, float ay, float az, float gx, float gy, float gz);
    void setTelemetryKinematics(float pitch, float pitchRate, uint32_t fault);
    void setTelemetryTuning(float kp, float ki, float k1, float k2, float k3);
    void setTelemetryQuat(float q0, float q1, float q2, float q3);
    void sendTelemetryPacket();

    // Setters for Slave -> Master (Commands)
    void setPI(float kp, float ki);
    void setLQRWeights(float k1, float k2, float k3);
    void setIMUTuning(float alpha, float accelTol);
    void setRobotState(uint8_t state, uint8_t edge);
    void setPitchTarget(float pitchTarget);
    void setMotorTargets(float t1, float t2, float t3);
    void sendCommandPacket(); 

    // Getters
    bool isConnectionAlive(uint32_t timeoutMs = 50); 
    uint32_t getFaultCode();
    float getMotorVel(int motor); 
    float getMotorCur(int motor);
    float getPitch();
    float getPitchRate();
    float getIMU(char axis, char type);
    
    float getKpOuter(); float getKiOuter();
    float getK1(); float getK2(); float getK3();
    float getBaseAlpha(); float getAccelTol();
    uint8_t getRobotState(); uint8_t getTargetEdge();
    float getTargetPitch();
    
    float getQ0(); float getQ1(); float getQ2(); float getQ3();
    
    float getActiveKp(); float getActiveKi();
    float getActiveK1(); float getActiveK2(); float getActiveK3();

private:
    HardwareSerial& _serial;
    PacketSerial _cobsSerial;
    int _rxPin, _txPin;
    std::atomic<uint32_t> _lastPacketTime;
    
    std::atomic<float> _t1{0}, _t2{0}, _t3{0}, _targetPitch{0};
    std::atomic<float> _kp{0}, _ki{0}, _k1{0}, _k2{0}, _k3{0};
    std::atomic<float> _alpha{0}, _accelTol{0};
    std::atomic<uint8_t> _robotState{0}, _targetEdge{0};

    std::atomic<uint32_t> _faultCode{0};
    std::atomic<float> _m1v{0}, _m2v{0}, _m3v{0}, _m1c{0}, _m2c{0}, _m3c{0};
    std::atomic<float> _ax{0}, _ay{0}, _az{0}, _gx{0}, _gy{0}, _gz{0};
    std::atomic<float> _pitch{0}, _pitchRate{0};
    std::atomic<float> _q0{1}, _q1{0}, _q2{0}, _q3{0};
    
    std::atomic<float> _act_kp{0}, _act_ki{0}, _act_k1{0}, _act_k2{0}, _act_k3{0};

    static void onPacketReceived(const uint8_t* buffer, size_t size);
};