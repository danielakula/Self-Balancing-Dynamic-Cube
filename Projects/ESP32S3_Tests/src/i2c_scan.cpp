#include <Arduino.h>
#include <Wire.h>

// Standard ESP32-S3 I2C pins
#define MASTER_SDA 4
#define MASTER_SCL 5

void setup() {
  // Use a high baud rate for clear output
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open

  pinMode(43, OUTPUT);
  digitalWrite(43, HIGH);
  delay(500);
  digitalWrite(43, LOW);
  delay(500);
  digitalWrite(43, HIGH);
  
  Serial.println("\n--- I2C Scanner Started ---");
  Serial.printf("SDA: %d | SCL: %d\n", MASTER_SDA, MASTER_SCL);

  // Initialize I2C
  Wire.begin(MASTER_SDA, MASTER_SCL);
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("Scan complete.\n");
  }

  delay(5000); // Wait 5 seconds for next scan
}