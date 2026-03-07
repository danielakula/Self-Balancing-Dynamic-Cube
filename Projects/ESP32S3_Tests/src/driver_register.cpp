#include <Arduino.h>
#include <SPI.h>

// --- PIN DEFINITIONS (Update based on your PCB) ---
#define DRV_CS   48
#define DRV_EN   35
#define DRV_MISO 39
#define DRV_MOSI 38
#define DRV_SCK  37
#define DRV_FAULT 45 // Connect to the nFAULT pin of the DRV

// --- DRV8323S REGISTER ADDRESSES ---
#define REG_FAULT_1  0x00
#define REG_VGS_STAT 0x01
#define REG_GATE_HS  0x03
#define REG_GATE_LS  0x04

// Helper function to handle the 16-bit SPI protocol
uint16_t drv_transfer(uint8_t address, bool isRead, uint16_t data) {
    // Structure: [1-bit R/W] [4-bit Address] [11-bit Data]
    uint16_t command = (isRead ? 1 : 0) << 15 | (address & 0x0F) << 11 | (data & 0x7FF);
    
    digitalWrite(DRV_CS, LOW);
    uint16_t response = SPI.transfer16(command);
    digitalWrite(DRV_CS, HIGH);
    
    return response & 0x7FF; // We only care about the lower 11 bits (data)
}

void setup() {
    Serial.begin(115200);
    while(!Serial); // Wait for Serial Monitor

    pinMode(DRV_CS, OUTPUT);
    pinMode(DRV_EN, OUTPUT);
    pinMode(DRV_FAULT, INPUT_PULLUP);
    digitalWrite(DRV_CS, HIGH);

    // 1. HARDWARE WAKEUP
    Serial.println("--- DRV8323S Handshake Initiation ---");
    digitalWrite(DRV_EN, HIGH);
    delay(50); // Minimum wake time is ~1ms, 50ms is safe for charge pump stability

    // 2. SPI SETUP (ESP32-S3 specific)
    // SPI Mode 1 is critical for DRV8323 (CPOL=0, CPHA=1)
    SPI.begin(DRV_SCK, DRV_MISO, DRV_MOSI, DRV_CS);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

    // 3. CONFIGURE IDRIVE (The "Handshake")
    // Target: 120mA P (0x7), 240mA N (0x7)
    // Register 0x03: [10:8]=Unlock(011), [7:4]=IDRIVEP, [3:0]=IDRIVEN
    uint16_t configValue = (0x3 << 8) | (0x7 << 4) | 0x7; 
    drv_transfer(REG_GATE_HS, false, configValue);
    
    // 4. VERIFICATION READBACK
    uint16_t readback = drv_transfer(REG_GATE_HS, true, 0x00);
    
    Serial.print("Target Config: 0x"); Serial.println(configValue, HEX);
    Serial.print("Actual Readback: 0x"); Serial.println(readback, HEX);

    if (readback == configValue) {
        Serial.println("SUCCESS: SPI Handshake verified!");
    } else {
        Serial.println("ERROR: SPI Mismatch. Check wiring/Logic levels.");
    }

    // 5. INITIAL FAULT CHECK
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
    // Continuously monitor the nFAULT pin
    if (digitalRead(DRV_FAULT) == LOW) {
        Serial.println("!!! HARDWARE FAULT TRIGGERED !!!");
        uint16_t f = drv_transfer(REG_FAULT_1, true, 0);
        Serial.print("Fault Detail: 0x"); Serial.println(f, HEX);
        delay(1000);
    }
}