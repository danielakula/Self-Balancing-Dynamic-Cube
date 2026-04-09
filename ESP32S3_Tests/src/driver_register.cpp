#include <Arduino.h>
#include <SPI.h>

#define DRV_CS   48
#define DRV_EN   35
#define DRV_MISO 39
#define DRV_MOSI 38
#define DRV_SCK  37
#define DRV_FAULT 45

#define REG_FAULT_1  0x00
#define REG_VGS_STAT 0x01
#define REG_GATE_HS  0x03
#define REG_GATE_LS  0x04

uint16_t drv_transfer(uint8_t address, bool isRead, uint16_t data) {

    uint16_t command = (isRead ? 1 : 0) << 15 | (address & 0x0F) << 11 | (data & 0x7FF);
    
    digitalWrite(DRV_CS, LOW);
    uint16_t response = SPI.transfer16(command);
    digitalWrite(DRV_CS, HIGH);
    
    return response & 0x7FF; 
}

void setup() {
    Serial.begin(115200);
    while(!Serial); 

    pinMode(DRV_CS, OUTPUT);
    pinMode(DRV_EN, OUTPUT);
    pinMode(DRV_FAULT, INPUT_PULLUP);
    digitalWrite(DRV_CS, HIGH);

    Serial.println("--- DRV8323S Handshake Initiation ---");
    digitalWrite(DRV_EN, HIGH);
    delay(50); 

    SPI.begin(DRV_SCK, DRV_MISO, DRV_MOSI, DRV_CS);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

    uint16_t configValue = (0x3 << 8) | (0x7 << 4) | 0x7; 
    drv_transfer(REG_GATE_HS, false, configValue);
    
    uint16_t readback = drv_transfer(REG_GATE_HS, true, 0x00);
    
    Serial.print("Target Config: 0x"); Serial.println(configValue, HEX);
    Serial.print("Actual Readback: 0x"); Serial.println(readback, HEX);

    if (readback == configValue) {
        Serial.println("SUCCESS: SPI Handshake verified!");
    } else {
        Serial.println("ERROR: SPI Mismatch. Check wiring/Logic levels.");
    }

    uint16_t fault1 = drv_transfer(REG_FAULT_1, true, 0);
    uint16_t fault2 = drv_transfer(REG_VGS_STAT, true, 0);
    
    if (fault1 == 0 && fault2 == 0) {
        Serial.println("HEALTH CHECK: No faults detected. PCB is safe.");
    } else {
        Serial.print("HEALTH CHECK: Fault detected! Reg0: 0x");
        Serial.print(fault1, HEX);
        Serial.print(" Reg1: 0x");
        Serial.println(fault2, HEX);
    }
}

void loop() {

    if (digitalRead(DRV_FAULT) == LOW) {
        Serial.println("!!! HARDWARE FAULT TRIGGERED !!!");
        uint16_t f = drv_transfer(REG_FAULT_1, true, 0);
        Serial.print("Fault Detail: 0x"); Serial.println(f, HEX);
        delay(1000);
    }
}