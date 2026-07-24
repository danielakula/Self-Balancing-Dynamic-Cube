#include "InterchipComms.h"

static InterchipComms* commsInstance = nullptr;

InterchipComms::InterchipComms(HardwareSerial& serialPort, int rxPin, int txPin)
    : _serial(serialPort), _rxPin(rxPin), _txPin(txPin) {
    _lastPacketTime.store(0);
}

void InterchipComms::begin(unsigned long baudRate) {
    commsInstance = this;
    _serial.begin(baudRate, SERIAL_8N1, _rxPin, _txPin);
    _cobsSerial.setStream(&_serial);
    _cobsSerial.setPacketHandler(&InterchipComms::onPacketReceived); 
}

void InterchipComms::update() {
    if (_serial.available() > 0) {
        _cobsSerial.update(); 
    }
}

// Setters (Commands) 
void InterchipComms::setPI(float kp, float ki) { _kp.store(kp); _ki.store(ki); }
void InterchipComms::setLQRWeights(float k1, float k2, float k3) { _k1.store(k1); _k2.store(k2); _k3.store(k3); }
void InterchipComms::setIMUTuning(float alpha, float accelTol) { _alpha.store(alpha); _accelTol.store(accelTol); }
void InterchipComms::setRobotState(uint8_t state, uint8_t edge) { _robotState.store(state); _targetEdge.store(edge); }
void InterchipComms::setPitchTarget(float pitchTarget) { _targetPitch.store(pitchTarget); }
void InterchipComms::setMotorTargets(float t1, float t2, float t3) { _t1.store(t1); _t2.store(t2); _t3.store(t3); }

//  Setters (Telemetry) 
void InterchipComms::setTelemetryMotors(float v1, float v2, float v3, float c1, float c2, float c3) {
    _m1v.store(v1); _m2v.store(v2); _m3v.store(v3);
    _m1c.store(c1); _m2c.store(c2); _m3c.store(c3);
}
void InterchipComms::setTelemetryIMU(float ax, float ay, float az, float gx, float gy, float gz) {
    _ax.store(ax); _ay.store(ay); _az.store(az);
    _gx.store(gx); _gy.store(gy); _gz.store(gz);
}
void InterchipComms::setTelemetryKinematics(float pitch, float pitchRate, uint32_t fault) {
    _pitch.store(pitch); _pitchRate.store(pitchRate); _faultCode.store(fault);
}
void InterchipComms::setTelemetryTuning(float kp, float ki, float k1, float k2, float k3) {
    _act_kp.store(kp); _act_ki.store(ki); _act_k1.store(k1); _act_k2.store(k2); _act_k3.store(k3);
}
void InterchipComms::setTelemetryQuat(float q0, float q1, float q2, float q3) {
    _q0.store(q0); _q1.store(q1); _q2.store(q2); _q3.store(q3);
}

//  Transmit 
void InterchipComms::sendCommandPacket() { 
    InterchipPacket p = {}; // Zero-initialise memory
    p.packetType = 0;       // Tag as COMMAND

    p.target1 = _t1.load(); p.target2 = _t2.load(); p.target3 = _t3.load(); p.targetPitch = _targetPitch.load();
    p.Kp_outer = _kp.load(); p.Ki_outer = _ki.load();
    p.k1 = _k1.load(); p.k2 = _k2.load(); p.k3 = _k3.load();
    p.BASE_ALPHA = _alpha.load(); p.ACCEL_TOLERANCE = _accelTol.load();
    p.robotState = _robotState.load(); p.targetEdge = _targetEdge.load();
    
    _cobsSerial.send((uint8_t*)&p, sizeof(InterchipPacket));
}

void InterchipComms::sendTelemetryPacket() { 
    InterchipPacket p = {}; // Zero-initialise memory
    p.packetType = 1;       // Tag as TELEMETRY

    p.faultCode = _faultCode.load();
    p.motor1Velocity = _m1v.load(); p.motor2Velocity = _m2v.load(); p.motor3Velocity = _m3v.load();
    p.motor1Current = _m1c.load(); p.motor2Current = _m2c.load(); p.motor3Current = _m3c.load();
    p.ax = _ax.load(); p.ay = _ay.load(); p.az = _az.load();
    p.gx = _gx.load(); p.gy = _gy.load(); p.gz = _gz.load();
    p.pitch = _pitch.load(); p.pitchRate = _pitchRate.load();
    p.active_Kp_outer = _act_kp.load(); p.active_Ki_outer = _act_ki.load();
    p.active_k1 = _act_k1.load(); p.active_k2 = _act_k2.load(); p.active_k3 = _act_k3.load();
    p.q0 = _q0.load(); p.q1 = _q1.load(); p.q2 = _q2.load(); p.q3 = _q3.load();
    
    _cobsSerial.send((uint8_t*)&p, sizeof(InterchipPacket));
}

//  Receive 
void InterchipComms::onPacketReceived(const uint8_t* buffer, size_t size) {
    if (commsInstance == nullptr) return; 
    if (size == sizeof(InterchipPacket)) {
        InterchipPacket p;
        memcpy(&p, buffer, size);
        
        if (p.packetType == 0) {
            // SLAVE -> MASTER: Update Commands ONLY
            commsInstance->_t1.store(p.target1); commsInstance->_t2.store(p.target2); commsInstance->_t3.store(p.target3);
            commsInstance->_targetPitch.store(p.targetPitch);
            commsInstance->_kp.store(p.Kp_outer); commsInstance->_ki.store(p.Ki_outer);
            commsInstance->_k1.store(p.k1); commsInstance->_k2.store(p.k2); commsInstance->_k3.store(p.k3);
            commsInstance->_alpha.store(p.BASE_ALPHA); commsInstance->_accelTol.store(p.ACCEL_TOLERANCE);
            commsInstance->_robotState.store(p.robotState); commsInstance->_targetEdge.store(p.targetEdge);
        } 
        else if (p.packetType == 1) {
            // MASTER -> SLAVE: Update Telemetry ONLY
            commsInstance->_faultCode.store(p.faultCode);
            commsInstance->_m1v.store(p.motor1Velocity); commsInstance->_m2v.store(p.motor2Velocity); commsInstance->_m3v.store(p.motor3Velocity);
            commsInstance->_m1c.store(p.motor1Current); commsInstance->_m2c.store(p.motor2Current); commsInstance->_m3c.store(p.motor3Current);
            commsInstance->_ax.store(p.ax); commsInstance->_ay.store(p.ay); commsInstance->_az.store(p.az);
            commsInstance->_gx.store(p.gx); commsInstance->_gy.store(p.gy); commsInstance->_gz.store(p.gz);
            commsInstance->_pitch.store(p.pitch); commsInstance->_pitchRate.store(p.pitchRate);
            
            commsInstance->_act_kp.store(p.active_Kp_outer); commsInstance->_act_ki.store(p.active_Ki_outer);
            commsInstance->_act_k1.store(p.active_k1); commsInstance->_act_k2.store(p.active_k2); commsInstance->_act_k3.store(p.active_k3);
            
            commsInstance->_q0.store(p.q0); commsInstance->_q1.store(p.q1); 
            commsInstance->_q2.store(p.q2); commsInstance->_q3.store(p.q3);
        }

        commsInstance->_lastPacketTime.store(millis()); 
    }
}

//  Getters 
bool InterchipComms::isConnectionAlive(uint32_t timeoutMs) { return (millis() - _lastPacketTime.load()) <= timeoutMs; }
uint32_t InterchipComms::getFaultCode() { return _faultCode.load(); }
float InterchipComms::getPitch() { return _pitch.load(); }
float InterchipComms::getPitchRate() { return _pitchRate.load(); }

float InterchipComms::getKpOuter() { return _kp.load(); }
float InterchipComms::getKiOuter() { return _ki.load(); }
float InterchipComms::getK1() { return _k1.load(); }
float InterchipComms::getK2() { return _k2.load(); }
float InterchipComms::getK3() { return _k3.load(); }
float InterchipComms::getBaseAlpha() { return _alpha.load(); }
float InterchipComms::getAccelTol() { return _accelTol.load(); }
uint8_t InterchipComms::getRobotState() { return _robotState.load(); }
uint8_t InterchipComms::getTargetEdge() { return _targetEdge.load(); }
float InterchipComms::getTargetPitch() { return _targetPitch.load(); }

float InterchipComms::getQ0() { return _q0.load(); }
float InterchipComms::getQ1() { return _q1.load(); }
float InterchipComms::getQ2() { return _q2.load(); }
float InterchipComms::getQ3() { return _q3.load(); }

float InterchipComms::getActiveKp() { return _act_kp.load(); }
float InterchipComms::getActiveKi() { return _act_ki.load(); }
float InterchipComms::getActiveK1() { return _act_k1.load(); }
float InterchipComms::getActiveK2() { return _act_k2.load(); }
float InterchipComms::getActiveK3() { return _act_k3.load(); }

float InterchipComms::getMotorCur(int motor) {
    if (motor == 1) return _m1c.load(); if (motor == 2) return _m2c.load(); if (motor == 3) return _m3c.load(); return 0;
}
float InterchipComms::getMotorVel(int motor) {
    if (motor == 1) return _m1v.load(); if (motor == 2) return _m2v.load(); if (motor == 3) return _m3v.load(); return 0;
}
float InterchipComms::getIMU(char axis, char type) {
    if (type == 'a') {
        if (axis == 'x') return _ax.load(); if (axis == 'y') return _ay.load(); if (axis == 'z') return _az.load();
    } else if (type == 'g') {
        if (axis == 'x') return _gx.load(); if (axis == 'y') return _gy.load(); if (axis == 'z') return _gz.load();
    }
    return 0.0f;
}