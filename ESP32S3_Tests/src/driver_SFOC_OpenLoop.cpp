#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

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

BLDCMotor motor = BLDCMotor(14); 
BLDCDriver6PWM driver = BLDCDriver6PWM(47, 21, 14, 13, 12, 11);

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
    
    digitalWrite(DRV_EN, HIGH);
    delay(50); 
    SPI.begin(DRV_SCK, DRV_MISO, DRV_MOSI, DRV_CS); 
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

    // REGISTERS
    writeSpi(0x02, 0b00000000000); 
    writeSpi(0x03, 0b01100100010); 
    writeSpi(0x04, 0b10100100010); 
    writeSpi(0x05, 0b01000010001); 
    writeSpi(0x06, 0b01101000011); 

    driver.voltage_power_supply = 15;
    driver.voltage_limit = 2.0;     
    driver.pwm_frequency = 20000;
    driver.init();
    
    motor.linkDriver(&driver);
    motor.voltage_limit = 0.75;      
    motor.velocity_limit = 10;     
    
    motor.controller = MotionControlType::velocity_openloop;

    motor.init();
    Serial.println("GL40 Ready. Sending move command...");
}

void loop() {
    if(digitalRead(nFAULT) == LOW) {
        driver.disable();
        Serial.println("!!! DRV FAULT !!!");
        while(1);
    }

    motor.move(5); // Target velocity
}