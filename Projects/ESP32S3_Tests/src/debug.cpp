#include <Arduino.h>
#include <SPI.h>

#define MASTER_SCK  37
#define MASTER_MISO 39
#define MASTER_MOSI 38
#define ENC1_CS     36

SPIClass hspi(HSPI);

void setup() {
    Serial.begin(115200);
    delay(2000); // Wait for Native USB
    Serial.println("\n--- Raw SPI Diagnostic Booting ---");

    // 1. Manually seize the CS pin from the OS
    pinMode(ENC1_CS, OUTPUT);
    digitalWrite(ENC1_CS, HIGH);

    // 2. Init SPI without passing the CS pin to the hardware matrix
    hspi.begin(MASTER_SCK, MASTER_MISO, MASTER_MOSI, -1);
}

void loop() {
    // 0xFFFF is the AS5047 command to read the Angle Register with Even Parity
    uint16_t command = 0xFFFF; 
    
    // AS5047 strictly requires SPI Mode 1
    hspi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    
    // Manually pulse CS
    digitalWrite(ENC1_CS, LOW);
    
    // Send command and read the 16-bit response simultaneously
    uint16_t response = hspi.transfer16(command);
    
    digitalWrite(ENC1_CS, HIGH);
    hspi.endTransaction();
    
    // The angle is the bottom 14 bits (Mask out the Parity and Error bits)
    uint16_t raw_angle = response & 0x3FFF;
    float angle_deg = ((float)raw_angle / 16384.0f) * 360.0f;

    Serial.printf("Raw Hex: 0x%04X | Angle: %6.2f deg\n", response, angle_deg);
    
    delay(100); // 10Hz print rate
}