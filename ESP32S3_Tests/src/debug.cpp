#include <Arduino.h>
#include <SPI.h>

#define MASTER_SCK  37
#define MASTER_MISO 39
#define MASTER_MOSI 38
#define ENC1_CS     36

SPIClass hspi(HSPI);

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n--- Raw SPI Diagnostic Booting ---");

    pinMode(ENC1_CS, OUTPUT);
    digitalWrite(ENC1_CS, HIGH);

    hspi.begin(MASTER_SCK, MASTER_MISO, MASTER_MOSI, -1);
}

void loop() {
    uint16_t command = 0xFFFF; 
    hspi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    digitalWrite(ENC1_CS, LOW);
    uint16_t response = hspi.transfer16(command);
    
    digitalWrite(ENC1_CS, HIGH);
    hspi.endTransaction();
    
    uint16_t raw_angle = response & 0x3FFF;
    float angle_deg = ((float)raw_angle / 16384.0f) * 360.0f;

    Serial.printf("Raw Hex: 0x%04X | Angle: %6.2f deg\n", response, angle_deg);
    
    delay(100);
}