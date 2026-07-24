#pragma once
#include <Arduino.h>

#ifndef _NC
#define _NC -1
#endif

#define WIFI_SSID "DAS"
#define WIFI_PASS "sharky96"
#define OTA_HOSTNAME "slave-mcu-drive"

enum SystemFault : uint32_t {
    FAULT_NONE          = 0,
    FAULT_UNDER_VOLTAGE = (1 << 0), 
    FAULT_OVER_VOLTAGE  = (1 << 1), 
    FAULT_COMMS_LOST    = (1 << 2), 
    FAULT_OTA_UPDATE    = (1 << 3), 
    FAULT_DRV1          = (1 << 4), 
    FAULT_DRV2          = (1 << 5), 
    FAULT_DRV3          = (1 << 6), 
    FAULT_KILLSWITCH    = (1 << 7), 
    FAULT_ROBOT_FALLEN  = (1 << 8),  
    FAULT_USER_DISABLE  = (1 << 9)   
};

constexpr float ANGLE_THRESHOLD = 0.35f; // 20 degrees
constexpr float MAX_PITCH_TARGET = 0.052f; // 3 degrees additional integral clamp
constexpr float INTEGRAL_CLAMPING = 500.0f;
constexpr float TASK_LOOP_FREQUENCY = 5.0f; // ms
constexpr uint32_t PRINT_INTERVAL_MS = 200; 

// Default LQR Weights
constexpr float DEFAULT_LQR_K1 = -75.6944f;  
constexpr float DEFAULT_LQR_K2 = -6.0373f;   
constexpr float DEFAULT_LQR_K3 = -0.0817f;   

// Default PI Gains
constexpr float DEFAULT_KP_OUTER = 0.003f; 
constexpr float DEFAULT_KI_OUTER = 0.0001f;

// Hardware Pins
constexpr int MASTER_SCK  = 37;
constexpr int MASTER_MISO = 39;
constexpr int MASTER_MOSI = 38;

constexpr int SLAVE_SCK   = 4;
constexpr int SLAVE_MISO  = 6;
constexpr int SLAVE_MOSI  = 5;

constexpr int MASTER_TX = 42;
constexpr int MASTER_RX = 41;
constexpr int SLAVE_TX  = 38;
constexpr int SLAVE_RX  = 37;

constexpr int SLAVE_UART_TX = 43;
constexpr int SLAVE_UART_RX = 44;

constexpr int ISM_CS = 16;

constexpr int VSENSE_PIN = 8;
constexpr float VSENSE_DIVIDER_RATIO = 11.0f; 
constexpr float VSENSE_TRIM = 1.008338f;

constexpr float BATTERY_SAFE_MAX = 16.8f; 
constexpr float BATTERY_SAFE_MIN = 14.0f; 

struct MotorPins {
    int enc_cs, drv_cs, drv_en, nFault; 
    int ah, al, bh, bl, ch, cl;         
    int soa, sob, soc;                  
};

inline constexpr MotorPins Motor1Pins = {
    36, 48, 35, 45,             
    47, 21, 14, 13, 12, 11,     
    3, 9, 10                    
};

inline constexpr MotorPins Motor2Pins = {
    39, 35, 46, 45,             
    48, 47, 21, 14, 13, 12,     
    1, 2, _NC                   
};

inline constexpr MotorPins Motor3Pins = {
    41, 10, 20, 19,             
    8, 18, 17, 16, 15, 7,       
    11, 9, 3                    
};

struct MotorConfig {
    float R, L, bandwidth_hz, driver_frequency, voltage_limit, current_limit, friction_comp;
};

inline constexpr MotorConfig MotorTuning = {
    0.537f, 0.00018f, 150.0f, 15000.0f, 14.0f, 3.0f, 0.0f        
};