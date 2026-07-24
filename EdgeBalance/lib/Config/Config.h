// ================================================================
// CONFIGURATION FILE - /w CUSTOMISABLE PARAMETERS
// ================================================================

#pragma once
#include <Arduino.h>
#include <SimpleFOC.h> // REQUIRED for the _NC (Not Connected) macro

// ================================================================
// FAULT REGISTERS (32-Bit Bitmask)
// ================================================================
enum SystemFault : uint32_t {
    FAULT_NONE          = 0,
    FAULT_UNDER_VOLTAGE = (1 << 0),  // 0x00000001
    FAULT_OVER_VOLTAGE  = (1 << 1),  // 0x00000002
    FAULT_COMMS_LOST    = (1 << 2),  // 0x00000004
    FAULT_OTA_UPDATE    = (1 << 3),  // 0x00000008
    FAULT_DRV1          = (1 << 4),  // 0x00000010
    FAULT_DRV2          = (1 << 5),  // 0x00000020
    FAULT_DRV3          = (1 << 6),  // 0x00000040
    FAULT_KILLSWITCH    = (1 << 7),  // 0x00000080
    FAULT_ROBOT_FALLEN  = (1 << 8),  // 0x00000100
    FAULT_USER_DISABLE  = (1 << 9)   // 0x00000200 
};

// ================================================================
// CONSTRAINTS & SYSTEM CONSTANTS
// ================================================================
constexpr float ANGLE_THRESHOLD = 20.0f * (PI / 180.0f); // activation angle
constexpr float MAX_PITCH_TARGET = 3.0f * (PI / 180.0f); // max target angle deviation
constexpr float INTEGRAL_CLAMPING = 500.0f; // MAX INTEGRAL = MAX TARGET ANGLE / Ki
constexpr float TASK_LOOP_FREQUENCY = 5.0f; // ms
constexpr uint32_t PRINT_INTERVAL_MS = 200; 

// ================================================================
// DEFAULT CONTROLLER TUNING
// ================================================================
// LQR Weights
constexpr float DEFAULT_LQR_K1 = -75.6944f;  // Cube Angle (Theta)
constexpr float DEFAULT_LQR_K2 = -6.0373f;   // Cube Velocity (Theta_dot)
constexpr float DEFAULT_LQR_K3 = -0.0817f;   // Wheel Velocity

// OUTER LOOP PI Gains
constexpr float DEFAULT_KP_OUTER = 0.003f; 
constexpr float DEFAULT_KI_OUTER = 0.0001f;

// ================================================================
// WIFI & OTA
// ================================================================
#define WIFI_SSID "DAS"
#define WIFI_PASS "sharky96"
#define OTA_HOSTNAME "slave-mcu-drive"

// ================================================================
// COMMUNICATION PROTOCOL PINS
// ================================================================
// SPI PINS MASTER
constexpr int MASTER_SCK  = 37;
constexpr int MASTER_MISO = 39;
constexpr int MASTER_MOSI = 38;

// SPI PINS SLAVE
constexpr int SLAVE_SCK   = 4;
constexpr int SLAVE_MISO  = 6;
constexpr int SLAVE_MOSI  = 5;

// INTERCHIP UART (Master <-> Slave)
constexpr int MASTER_TX = 42;
constexpr int MASTER_RX = 41;
constexpr int SLAVE_TX  = 38;
constexpr int SLAVE_RX  = 37;

// UART SLAVE
constexpr int SLAVE_UART_TX = 43;
constexpr int SLAVE_UART_RX = 44;

// IMU
constexpr int ISM_CS = 16;

// ================================================================
// VOLTAGE MONITOR CONFIGURATION
// ================================================================
constexpr int VSENSE_PIN = 8;
constexpr float VSENSE_DIVIDER_RATIO = 11.0f; 
constexpr float VSENSE_TRIM = 1.008338f;

constexpr float BATTERY_SAFE_MAX = 16.8f; // 4S LiPo MAX
constexpr float BATTERY_SAFE_MIN = 14.0f; // 4S LiPo MIN

// ================================================================
//  MOTOR PINS & HARDWARE CONFIG
// ================================================================
struct MotorPins {
    int enc_cs, drv_cs, drv_en, nFault; // 4 Control Pins
    int ah, al, bh, bl, ch, cl;         // 6 PWM Pins
    int soa, sob, soc;                  // 3 Current Sense Pins
};

// MOTOR 1 (Master MCU) - Must use 'inline constexpr' to prevent linker errors
inline constexpr MotorPins Motor1Pins = {
    36, 48, 35, 45,             // ENC_CS, DRV_CS, EN, Fault
    47, 21, 14, 13, 12, 11,     // PWM (AH..CL)
    3, 9, 10                    // Current Sense (SoA, SoB, SoC)
};

// MOTOR 2 (Slave MCU - Left)
inline constexpr MotorPins Motor2Pins = {
    39, 35, 46, 45,             // ENC_CS, DRV_CS, EN, Fault
    48, 47, 21, 14, 13, 12,     // PWM (AH..CL)
    1, 2, _NC                   // Current Sense (SoA, SoB, _NC)
};

// MOTOR 3 (Slave MCU - Right)
inline constexpr MotorPins Motor3Pins = {
    41, 10, 20, 19,             // ENC_CS, DRV_CS, EN, Fault
    8, 18, 17, 16, 15, 7,       // PWM (AH..CL)
    11, 9, 3                    // Current Sense (SoA, SoB, SoC)
};

// ================================================================
//  MOTOR TUNING
// ================================================================
struct MotorConfig {
    float R, L;
    float bandwidth_hz;
    float driver_frequency;
    float voltage_limit;
    float current_limit;
    float friction_comp;
};

inline constexpr MotorConfig MotorTuning = {
    0.537f,     // R (Resistance) 
    0.00018f,   // L (Inductance)
    150.0f,     // Bandwidth (Hz)
    15000.0f,   // Driver frequency (Hz)
    14.0f,      // Voltage limit (Volts)
    3.0f,       // Current limit (Amps)
    0.0f        // Static friction compensation (Amps) roughly 0.15A - work well at 0.0A
};