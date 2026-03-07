#include <Arduino.h>

// Initialize UART1
HardwareSerial InterChipSerial(1); 

#define INTERCHIP_MOSI 37
#define INTERCHIP_MISO 38

void setup() {
  Serial.begin(115200); // Debug serial to your computer
  
  // Begin UART1 with the slave's pins
  InterChipSerial.begin(115200, SERIAL_8N1, INTERCHIP_MOSI, INTERCHIP_MISO);
  
  Serial.println("Slave MCU Ready. Listening for Master...");
}

void loop() {
  // Listen for incoming UART messages
  if (InterChipSerial.available()) {
    String msg = InterChipSerial.readStringUntil('\n');
    msg.trim(); // Clean up the string
    
    if (msg == "PING") {
      Serial.println("Slave: Received PING, sending PONG...");
      // Reply to the master
      InterChipSerial.println("PONG"); 
    } else {
      Serial.print("Slave: Received unknown command: ");
      Serial.println(msg);
    }
  }
}