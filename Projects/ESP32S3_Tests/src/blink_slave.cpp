#include <Arduino.h>

// put function declarations here:

void setup() {
    Serial.begin(115200);
  pinMode(40, OUTPUT);
}

void loop() {
  digitalWrite(40, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(150);            // wait for a second
  digitalWrite(40, LOW); // turn the LED off by making the voltage LOW
  delay(150);            // wait for a second
  Serial.println("Blinking...");
}