#pragma once
#include <Arduino.h>
#include <SimpleFOC.h> // REQUIRED for the _NC (Not Connected) macro!

// ================================================================
// WIFI & OTA
// ================================================================
#define WIFI_SSID "DAS"
#define WIFI_PASS "sharky96"
#define OTA_HOSTNAME "slave-mcu-drive"

// ================================================================
// FAULT REGISTERS (32-Bit Bitmask)
// ================================================================
enum SystemFault : uint32_t {
    FAULT_NONE          = 0x00000000,
    
    // Power Faults
    FAULT_UNDER_VOLTAGE = 0x00000001, 
    FAULT_OVER_VOLTAGE  = 0x00000002, 
    
    // Communication Faults
    FAULT_COMMS_LOST    = 0x00000004, 
    FAULT_OTA_UPDATE    = 0x00000008, 
    
    // Hardware Faults
    FAULT_DRV1          = 0x00000010, 
    FAULT_DRV2          = 0x00000020, 
    FAULT_DRV3          = 0x00000040  
};

// ================================================================
// COMMUNICATION PROTOCOL PINS
// ================================================================
// SPI PINS MASTER
const int MASTER_SCK  = 37;
const int MASTER_MISO = 39;
const int MASTER_MOSI = 38;

// SPI PINS SLAVE
const int SLAVE_SCK   = 4;
const int SLAVE_MISO  = 6;
const int SLAVE_MOSI  = 5;

// INTERCHIP UART (Master <-> Slave)
const int MASTER_TX = 42;
const int MASTER_RX = 41;
const int SLAVE_TX  = 38;
const int SLAVE_RX  = 37;

// UART SLAVE
const int SLAVE_UART_TX = 43;
const int SLAVE_UART_RX = 44;

// ================================================================
//  MOTOR PINS
// ================================================================
struct MotorPins {
    int enc_cs, drv_cs, drv_en, nFault; // 4 Control Pins
    int ah, al, bh, bl, ch, cl;         // 6 PWM Pins
    int soa, sob, soc;                  // 3 Current Sense Pins
};

// MOTOR 1 (Master MCU)
const MotorPins Motor1Pins = {
    36, 48, 35, 45,             // ENC_CS, DRV_CS, EN, Fault
    47, 21, 14, 13, 12, 11,     // PWM (AH..CL)
    3, 9, 10                    // Current Sense (SoA, SoB, SoC)
};

// MOTOR 2 (Slave MCU - Left)
const MotorPins Motor2Pins = {
    39, 35, 46, 45,             // ENC_CS, DRV_CS, EN, Fault
    48, 47, 21, 14, 13, 12,     // PWM (AH..CL)
    1, 2, _NC                   // Current Sense (SoA, SoB, _NC)
};

// MOTOR 3 (Slave MCU - Right)
const MotorPins Motor3Pins = {
    41, 10, 20, 19,             // ENC_CS, DRV_CS, EN, Fault
    8, 18, 17, 16, 15, 7,       // PWM (AH..CL)
    11, 9, 3                    // Current Sense (SoA, SoB, SoC)
};

// ================================================================
// VOLTAGE MONITOR CONFIGURATION
// ================================================================
const int VSENSE_PIN = 8;
const float VSENSE_DIVIDER_RATIO = 11.0f; 
const float VSENSE_TRIM = 1.008338f;
const float BATTERY_SAFE_MAX = 16.8f; // 4S LiPo MAX
const float BATTERY_SAFE_MIN = 14.2f; // 4S LiPo MIN

// ================================================================
// TUNING CONFIGURATIONS
// ================================================================
struct MotorConfig {
    float R, L;
    float bandwidth_hz;
    float driver_frequency;
    float voltage_limit;
    float current_limit;
};

const MotorConfig MotorTuning = {
    0.5f,   // R (Resistance) 
    0.00018f,  // L (Inductance)
    200.0f, // Bandwidth (Hz)
    12500.0f, // Driver frequency (Hz)
    14.0f,  // Voltage limit (Volts)
    1.0f    // Current limit (Amps) - Increase this safely when testing torque
};