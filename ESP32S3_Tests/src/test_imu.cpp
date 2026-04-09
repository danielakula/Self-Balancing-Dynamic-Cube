#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>

#define ISM_CS 16
#define SPI_SCK 37
#define SPI_MISO 39
#define SPI_MOSI 38

Adafruit_ISM330DHCX ism;

void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("ISM330DHCX SPI Test");

  if (!ism.begin_SPI(ISM_CS, SPI_SCK, SPI_MISO, SPI_MOSI)) {
    Serial.println("Failed to find ISM330DHCX chip! Check wiring.");
    while (1) delay(10);
  }

  Serial.println("ISM330DHCX Found!");

  // Set ranges
  ism.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  ism.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
}

void loop() {
  sensors_event_t accel, gyro, temp;
  ism.getEvent(&accel, &gyro, &temp);

  Serial.print(accel.acceleration.x,4);
  Serial.print(",");
  Serial.print(accel.acceleration.y,4);
  Serial.print(",");
  Serial.print(accel.acceleration.z,4);
  Serial.print(",");
  Serial.print(gyro.gyro.x,4);
  Serial.print(",");
  Serial.print(gyro.gyro.y,4);
  Serial.print(",");
  Serial.print(gyro.gyro.z,4);

  Serial.println();
  delay(40);
}