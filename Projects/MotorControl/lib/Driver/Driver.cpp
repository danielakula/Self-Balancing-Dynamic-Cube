#include "Driver.h"

// --------------------------------------------------------
// CONSTRUCTOR
// --------------------------------------------------------
Driver::Driver(const MotorPins& pins, const MotorConfig& config)
    : _pins(pins), _config(config), _target(0.0f), _enabled(true),
      _driver(pins.ah, pins.al, pins.bh, pins.bl, pins.ch, pins.cl),
      _motor(14, 0.5f, 195.0f, 0.00018f), 
      _sensor(AS5047_SPI, pins.enc_cs),
      _currentSense(0.010f, 20.0f, pins.soa, pins.sob, pins.soc) 
{}

// --------------------------------------------------------
// FAULT CHECKING (External logic handles SPI safety)
// --------------------------------------------------------
bool Driver::hasHardwareFault() {
    return (digitalRead(_pins.nFault) == LOW);
}

void Driver::printDetailedFaults(const char* motorName) {
    // We assume the caller (Master/Slave) has already secured the SPI bus!
    uint16_t faultReg1 = drvReadSpi(0x00); 
    uint16_t faultReg2 = drvReadSpi(0x01); 
    
    Serial.printf(">>> %s DRV8323 FAULT REPORT <<<\n", motorName);
    Serial.printf("Reg 0x00: 0x%03X | Reg 0x01: 0x%03X\n", faultReg1, faultReg2);
    
    if (faultReg1 & (1<<10)) Serial.println(" - FAULT: System Fault");
    if (faultReg1 & (1<<8))  Serial.println(" - GDF: Gate Drive Fault");
    if (faultReg1 & (1<<7))  Serial.println(" - UVLO: Undervoltage Lockout");
}

// --------------------------------------------------------
// SETUP: Configures hardware and initializes FOC
// --------------------------------------------------------
void Driver::begin(SPIClass* spiBus) {
    _spi = spiBus; 

    pinMode(_pins.drv_cs, OUTPUT); digitalWrite(_pins.drv_cs, HIGH);
    pinMode(_pins.enc_cs, OUTPUT); digitalWrite(_pins.enc_cs, HIGH);
    pinMode(_pins.drv_en, OUTPUT); digitalWrite(_pins.drv_en, LOW);
    delay(50);
    digitalWrite(_pins.drv_en, HIGH);
    delay(100);
    pinMode(_pins.nFault, INPUT_PULLUP);

    // Initialize DRV8323
    initDRV8323S();

    // 2MHz is the sweet spot for AS5047 signal integrity
    _sensor.init(_spi);

    // PWM Frequency at 15kHz gives the ADC more time to sample in a quiet window
    _driver.pwm_frequency = _config.driver_frequency;
    _driver.voltage_power_supply = 15.0f;
    _driver.voltage_limit = _config.voltage_limit;
    _driver.init();

    _currentSense.linkDriver(&_driver);

    // ADC Hardware Fix
    analogRead(_pins.soa);
    analogRead(_pins.sob);
    if (_pins.soc != -1) analogRead(_pins.soc); //NC check
    delay(10); 

    _currentSense.init();

    // Motor & PID Configuration
    _motor.linkSensor(&_sensor);
    _motor.linkDriver(&_driver);
    _motor.linkCurrentSense(&_currentSense);

    _motor.torque_controller = TorqueControlType::foc_current;
    _motor.controller = MotionControlType::torque;

    float omega = _2PI * _config.bandwidth_hz;
    float calc_P = _config.L * omega;
    float calc_I = _config.R * omega;
    float calc_Tf = 1.0 / (omega * 5.0);

    motor.PID_current_q.P = calc_P;
    motor.PID_current_q.I = calc_I;
    motor.PID_current_d.P = calc_P;
    motor.PID_current_d.I = calc_I;
    motor.LPF_current_q.Tf = calc_Tf; 
    motor.LPF_current_d.Tf = calc_Tf;

    _motor.current_limit = _config.current_limit;
    _motor.voltage_sensor_align = 1.0f;
    _motor.velocity_limit = 10.0f; 
    
    _motor.init();
    _motor.initFOC();
}

// --------------------------------------------------------
// REAL-TIME FOC LOOP (Runs on Core 1)
// --------------------------------------------------------
void Driver::runFOC() {
    if (!_enabled.load()) {
        _driver.disable();
        return;
    }

    // Mutex removed! Core-1-specific Spinlock in main.cpp handles bus safety.
    _motor.loopFOC();
    _motor.move(_target.load());
}

// --------------------------------------------------------
// SAFETY & CONTROL (Atomic operations)
// --------------------------------------------------------
void Driver::setTarget(float targetCurrent) { _target.store(targetCurrent); }
void Driver::emergencyStop() { _enabled.store(false); }
void Driver::enable() { _enabled.store(true); _driver.enable(); }

float Driver::getVelocity() { return _motor.shaft_velocity; }
float Driver::getCurrentQ() { return _motor.current.q; }

// --------------------------------------------------------
// LOW-LEVEL SPI (Optimized for Speed)
// --------------------------------------------------------
void Driver::drvWriteSpi(uint8_t address, uint16_t data) {
    digitalWrite(_pins.enc_cs, HIGH); 
    uint16_t frame = ((uint16_t)(address & 0x0F) << 11) | (data & 0x7FF);
    _spi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
    digitalWrite(_pins.drv_cs, LOW);
    delayMicroseconds(1);
    _spi->transfer16(frame);
    delayMicroseconds(1);
    digitalWrite(_pins.drv_cs, HIGH);
    _spi->endTransaction();
}

uint16_t Driver::drvReadSpi(uint8_t address) {
    digitalWrite(_pins.enc_cs, HIGH); 
    uint16_t frame = (1 << 15) | ((uint16_t)(address & 0x0F) << 11);
    _spi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
    digitalWrite(_pins.drv_cs, LOW);
    delayMicroseconds(1);
    uint16_t result = _spi->transfer16(frame);
    delayMicroseconds(1);
    digitalWrite(_pins.drv_cs, HIGH);
    _spi->endTransaction();
    return result & 0x7FF;
}

void Driver::initDRV8323S() {
    drvWriteSpi(0x02, 0b00000000000); // IC Control
    drvWriteSpi(0x03, 0b01100100010); // Gate Drive HS
    drvWriteSpi(0x04, 0b10100100010); // Gate Drive LS
    drvWriteSpi(0x05, 0b01000011001); // OCP Control
    drvWriteSpi(0x06, 0b01110000011); // CSA Control
}