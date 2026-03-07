#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WebServer.h>

const char *ssid     = "DAS"; //Enter the router name
const char *password = "sharky96"; //Enter the router password

WiFiServer telnetServer(23);
WiFiClient telnetClient;

WebServer server(80); // Create a web server on port 80

void handleRoot() {
  server.send(200, "text/plain", "Hello from ESP32-S3! Wireless communication established.");
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Set up the "Wireless Communication" endpoint
  server.on("/", handleRoot);
  server.begin();
  Serial.println("HTTP server started");

  ArduinoOTA.begin();

  telnetServer.begin();
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient(); // This keeps the web server alive

  if (telnetServer.hasClient()) {
    if (telnetClient) telnetClient.stop();
    telnetClient = telnetServer.available();
    telnetClient.println("Wireless Serial Monitor Connected!");
  }
}