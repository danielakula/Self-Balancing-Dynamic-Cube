#include <Arduino.h>

void setup() {
  pinMode(43, OUTPUT);
}

void loop() {
  digitalWrite(43, HIGH); 
  delay(50);            
  digitalWrite(43, LOW); 
  delay(50);            
}
