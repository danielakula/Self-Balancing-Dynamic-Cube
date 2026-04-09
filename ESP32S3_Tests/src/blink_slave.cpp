#include <Arduino.h>

void setup() {
    Serial.begin(115200);
  pinMode(40, OUTPUT);
}

void loop() {
  digitalWrite(40, HIGH); 
  delay(150);            
  digitalWrite(40, LOW); 
  delay(150);            
  Serial.println("Blinking...");
}