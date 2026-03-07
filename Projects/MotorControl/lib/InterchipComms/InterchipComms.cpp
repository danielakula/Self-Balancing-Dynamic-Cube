#include "InterchipComms.h"

// Static pointer so the library callback knows which object to update
static InterchipComms* commsInstance = nullptr;

// --------------------------------------------------------
// CONSTRUCTOR
// --------------------------------------------------------
InterchipComms::InterchipComms(HardwareSerial& serialPort, int rxPin, int txPin)
    : _serial(serialPort), _rxPin(rxPin), _txPin(txPin), 
      _latestTarget(0.0f), _remoteFaultCode(0), _lastPacketTime(0),
      _m2Vel(0.0f), _m3Vel(0.0f), _m2Cur(0.0f), _m3Cur(0.0f) {}

// --------------------------------------------------------
// SETUP & UPDATE
// --------------------------------------------------------
void InterchipComms::begin(unsigned long baudRate) {
    commsInstance = this; // Link the pointer to this specific object
    
    _serial.begin(baudRate, SERIAL_8N1, _rxPin, _txPin);
    _cobsSerial.setStream(&_serial);
    
    // Use the standard function name the library expects
    _cobsSerial.setPacketHandler(&InterchipComms::onPacketReceived); 
}

void InterchipComms::update() {
    _cobsSerial.update();
}

// --------------------------------------------------------
// SENDING DATA
// --------------------------------------------------------
void InterchipComms::setTelemetry(float m2Vel, float m3Vel, float m2Cur, float m3Cur) {
    _m2Vel.store(m2Vel);
    _m3Vel.store(m3Vel);
    _m2Cur.store(m2Cur);
    _m3Cur.store(m3Cur);
}

void InterchipComms::sendPacket(float target, uint32_t faultCode) {
    InterchipPacket packet;
    packet.targetCurrent = target;
    packet.faultCode = faultCode;
    
    // Load local telemetry state into the packet before sending
    packet.motor2Velocity = _m2Vel.load();
    packet.motor3Velocity = _m3Vel.load();
    packet.motor2Current = _m2Cur.load();
    packet.motor3Current = _m3Cur.load();

    _cobsSerial.send((uint8_t*)&packet, sizeof(InterchipPacket));
}

// --------------------------------------------------------
// GETTERS & SAFETY
// --------------------------------------------------------
float InterchipComms::getTargetCurrent() { return _latestTarget.load(); }
uint32_t InterchipComms::getRemoteFaultCode() { return _remoteFaultCode.load(); }

float InterchipComms::getMotor2Velocity() { return _m2Vel.load(); }
float InterchipComms::getMotor3Velocity() { return _m3Vel.load(); }
float InterchipComms::getMotor2Current() { return _m2Cur.load(); }
float InterchipComms::getMotor3Current() { return _m3Cur.load(); }

bool InterchipComms::isConnectionAlive(uint32_t timeoutMs) {
    if (_lastPacketTime.load() == 0) return false; 
    return (millis() - _lastPacketTime.load()) <= timeoutMs;
}

// --------------------------------------------------------
// RECEIVE CALLBACK
// --------------------------------------------------------
void InterchipComms::onPacketReceived(const uint8_t* buffer, size_t size) {
    // Safety check: make sure the object exists
    if (commsInstance == nullptr) return; 

    if (size == sizeof(InterchipPacket)) {
        InterchipPacket packet;
        memcpy(&packet, buffer, size);
        
        // Update targets and faults using our new static pointer
        commsInstance->_latestTarget.store(packet.targetCurrent);
        commsInstance->_remoteFaultCode.store(packet.faultCode);
        
        // Update telemetry data
        commsInstance->_m2Vel.store(packet.motor2Velocity);
        commsInstance->_m3Vel.store(packet.motor3Velocity);
        commsInstance->_m2Cur.store(packet.motor2Current);
        commsInstance->_m3Cur.store(packet.motor3Current);
        
        // Reset watchdog timer
        commsInstance->_lastPacketTime.store(millis()); 
    }
}