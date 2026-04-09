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

// 6-PWM Hardware Pins
#define AH 47  
#define AL 21
#define BH 14
#define BL 13
#define CH 12
#define CL 11

// DRIVER INSTANCE
BLDCDriver6PWM driver = BLDCDriver6PWM(AH, AL, BH, BL, CH, CL);

// Write to DRV8323
void writeSpi(uint8_t regAddr, uint16_t regVal) {
    uint16_t packet = (0 << 15) | (regAddr & 0x0F) << 11 | (regVal & 0x7FF);
    digitalWrite(DRV_CS, LOW);
    SPI.transfer16(packet);
    digitalWrite(DRV_CS, HIGH);
}

// Read from DRV8323
uint16_t readSpi(uint8_t regAddr) {
    uint16_t packet = (1 << 15) | (regAddr & 0x0F) << 11 | 0x00;
    digitalWrite(DRV_CS, LOW);
    uint16_t response = SPI.transfer16(packet);
    digitalWrite(DRV_CS, HIGH);
    return response & 0x07FF; // Mask to 11 bits
}

void setup() {
    Serial.begin(115200);
    while(!Serial); // Wait for Serial Monitor

    pinMode(nFAULT, INPUT_PULLUP);
    pinMode(DRV_CS, OUTPUT);
    pinMode(DRV_EN, OUTPUT);
    
    digitalWrite(DRV_EN, HIGH);
    delay(50); 

    SPI.begin(DRV_SCK, DRV_MISO, DRV_MOSI, DRV_CS); 
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

    // BINARY REGISTERS
    writeSpi(0x02, 0b00000000000); 
    writeSpi(0x03, 0b01100100010); 
    writeSpi(0x04, 0b10100100010); 
    writeSpi(0x05, 0b01000010001); 
    writeSpi(0x06, 0b01101000011); 

    // VERIFICATION READBACK
    Serial.println("--- DRV8323 REGISTER VERIFICATION ---");
    for(uint8_t i = 0x02; i <= 0x06; i++) {
        uint16_t val = readSpi(i);
        Serial.print("Reg 0x"); Serial.print(i, HEX);
        Serial.print(": 0b"); Serial.println(val, BIN);
    }
    Serial.println("-------------------------------------");

    driver.voltage_power_supply = 15;
    driver.voltage_limit = 6.0;    // Safety cap for open loop
    driver.pwm_frequency = 20000; 
    
    if (driver.init()) Serial.println("Driver Init Success!");
    else Serial.println("Driver Init Failed!");

    driver.enable();
    Serial.println("Starting Open-Loop Sine Injection...");
}

void loop() {
    if(digitalRead(nFAULT) == LOW) {
        driver.disable();
        Serial.print("!!! DRV FAULT !!! Status Reg 0x00: 0b");
        Serial.println(readSpi(0x00), BIN);
        while(1); 
    }

    static float angle = 0;
    float v_test = 5.0; 
    
    float v_a = v_test * sin(angle);
    float v_b = v_test * sin(angle + (2.0f * PI / 3.0f));
    float v_c = v_test * sin(angle + (4.0f * PI / 3.0f));

    driver.setPwm(v_a, v_b, v_c);
    
    angle += 0.02f; 

    static unsigned long lastTime = 0;
    if(millis() - lastTime > 1000) {
        Serial.println("Wiggling phases... Status OK.");
        lastTime = millis();
    }
    
    delay(1); 
}