#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

void setup() {
  Serial.begin(115200);
  WiFi.begin("DAS", "sharky96");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // OTA Initialisation 
  ArduinoOTA.setHostname("my-esp32-device"); 
  // ArduinoOTA.setPassword("admin"); // Add a password for security

  ArduinoOTA.onStart([]() {
    Serial.println("Start updating...");
  });
  
  ArduinoOTA.begin();
  Serial.println("Ready for OTA!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  ArduinoOTA.handle(); // This is MANDATORY
  // other code 
}
