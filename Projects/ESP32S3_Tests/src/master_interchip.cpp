#include <Arduino.h>

// Initialize UART1
HardwareSerial InterChipSerial(1); 

#define INTERCHIP_MISO 41
#define INTERCHIP_MOSI 42

void setup() {
  Serial.begin(115200); // Debug serial to your computer
  
  // Begin UART1 at 115200 baud, 8 data bits, no parity, 1 stop bit
  InterChipSerial.begin(115200, SERIAL_8N1, INTERCHIP_MISO, INTERCHIP_MOSI);
  
  Serial.println("Master MCU Ready. Starting Ping-Pong test...");
}

void loop() {
  // 1. Send a message to the slave
  InterChipSerial.println("PING");
  Serial.println("Master: Sent PING");

  // 2. Wait a moment for the slave to process and reply
  delay(50); 

  // 3. Check for a reply
  while (InterChipSerial.available()) {
    String msg = InterChipSerial.readStringUntil('\n');
    msg.trim(); // Clean up any trailing \r
    Serial.print("Master received: ");
    Serial.println(msg);
  }
  
  delay(1000); // Send a ping every 1 second
}