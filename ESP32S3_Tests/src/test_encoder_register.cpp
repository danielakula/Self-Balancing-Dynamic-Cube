#include <Arduino.h>
#include <SPI.h>

// Shared SPI Pins
#define MASTER_SCK  37
#define MASTER_MISO 39
#define MASTER_MOSI 38
#define ENC1_CS     36

void setup() {
    Serial.begin(115200);
    delay(2000);

    // FORCE ALL CS PINS HIGH IMMEDIATELY
    pinMode(ENC1_CS, OUTPUT);
    digitalWrite(ENC1_CS, HIGH);

    // Initialize SPI
    SPI.begin(MASTER_SCK, MASTER_MISO, MASTER_MOSI);
    Serial.println("System Ready. Testing AS5047P...");
}

uint16_t transfer(uint16_t cmd) {
    // use a slow speed and MODE 1
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    digitalWrite(ENC1_CS, LOW);
    
    uint16_t result = SPI.transfer16(cmd);
    
    digitalWrite(ENC1_CS, HIGH);
    SPI.endTransaction();
    return result;
}

void loop() {
  // prepare the bus for the Encoder (Mode 1)
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ENC1_CS, LOW);
  
  // Send 0xFFFF (Read Angle)
  uint16_t response = SPI.transfer16(0xFFFF);
  
  digitalWrite(ENC1_CS, HIGH);
  SPI.endTransaction();

  // Report
  if (response == 0xFFFF) {
    Serial.println("Encoder is SILENT (MISO is stuck HIGH)");
  } else if (response == 0x0000) {
    Serial.println("Encoder is SILENT (MISO is stuck LOW)");
  } else {
    Serial.print("Encoder responded! Raw: ");
    Serial.println(response, HEX);
  }

  delay(500);
}