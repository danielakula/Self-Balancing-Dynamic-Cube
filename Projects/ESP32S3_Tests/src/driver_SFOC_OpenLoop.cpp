#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

// --- PIN DEFINITIONS ---
#define DRV_CS    48
#define DRV_EN    35
#define DRV_MISO  39
#define DRV_MOSI  38
#define DRV_SCK   37
#define nFAULT    45 

// 6-PWM Pins
#define AH 47, AL 21
#define BH 14, BL 13
#define CH 12, CL 11

// --- INSTANCES ---
// CubeMars GL40 has 14 pole pairs
BLDCMotor motor = BLDCMotor(14); 
BLDCDriver6PWM driver = BLDCDriver6PWM(47, 21, 14, 13, 12, 11);

// --- SPI HELPER ---
void writeSpi(uint8_t regAddr, uint16_t regVal) {
    uint16_t packet = (0 << 15) | (regAddr & 0x0F) << 11 | (regVal & 0x7FF);
    digitalWrite(DRV_CS, LOW);
    SPI.transfer16(packet);
    digitalWrite(DRV_CS, HIGH);
}

void setup() {
    Serial.begin(115200);
    while(!Serial);
    
    pinMode(nFAULT, INPUT_PULLUP);
    pinMode(DRV_CS, OUTPUT);
    pinMode(DRV_EN, OUTPUT);
    
    // 1. Wake & SPI Init
    digitalWrite(DRV_EN, HIGH);
    delay(50); 
    SPI.begin(DRV_SCK, DRV_MISO, DRV_MOSI, DRV_CS); 
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

    // 2. APPLY YOUR VERIFIED REGISTERS
    writeSpi(0x02, 0b00000000000); 
    writeSpi(0x03, 0b01100100010); 
    writeSpi(0x04, 0b10100100010); 
    writeSpi(0x05, 0b01000010001); 
    writeSpi(0x06, 0b01101000011); 

    // 3. DRIVER CONFIG
    driver.voltage_power_supply = 15;
    driver.voltage_limit = 2.0;     // Increase to 5V for GL40 torque
    driver.pwm_frequency = 20000;
    driver.init();
    
    // 4. MOTOR CONFIG
    motor.linkDriver(&driver);
    motor.voltage_limit = 0.75;      // Safety limit inside the motor logic
    motor.velocity_limit = 10;      // rad/s (approx 95 RPM)
    
    // Set open-loop mode
    motor.controller = MotionControlType::velocity_openloop;

    motor.init();
    Serial.println("GL40 Ready. Sending move command...");
}

void loop() {
    // A. Check for DRV Faults
    if(digitalRead(nFAULT) == LOW) {
        driver.disable();
        Serial.println("!!! DRV FAULT !!!");
        while(1);
    }

    // B. SimpleFOC Open-Loop Move
    // This handles all the sine wave math automatically
    motor.move(5); // Target velocity = 5 radians per second
}