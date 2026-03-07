#include <Arduino.h>

#define VSENSE_PIN 8      // GPIO 8 (ADC1_CH7)
#define SAMPLES 64        
const float dividerRatio = 11.0; 
const float calibrationTrim = 1.008338;

void setup() {
  Serial.begin(115200);
  
  // Set resolution to 12-bit (0-4095)
  analogReadResolution(12);
  
  // Important: Set attenuation for the pin to handle the ~1.5V signal
  // ADC_11db allows for a range up to ~3.1V
  analogSetAttenuation(ADC_11db);
}

void loop() {
  uint32_t totalMilliVolts = 0;
  
  for (int i = 0; i < SAMPLES; i++) {
    // analogReadMilliVolts handles the eFuse calibration and Vref logic automatically!
    totalMilliVolts += analogReadMilliVolts(VSENSE_PIN);
    delay(1); // Small delay to let ADC stabilize
  }
  
  float avgMilliVolts = (float)totalMilliVolts / SAMPLES;

  // Scale by your 11:1 divider (100k + 10k) / 10k
  float batteryVoltage = (avgMilliVolts / 1000.0) * dividerRatio * calibrationTrim;

  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage, 4);
  Serial.println(" V");

  delay(1000);
}