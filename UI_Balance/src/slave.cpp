#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "InterchipComms.h"
#include "Config.h"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
InterchipComms comms(Serial1, SLAVE_RX, SLAVE_TX);

unsigned long lastTelemetryTime = 0;

static uint32_t lastHeartbeat = 0;

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            StaticJsonDocument<1024> doc;
            DeserializationError error = deserializeJson(doc, data, len);
            if (error) return;

            const char* cmd = doc["cmd"];
            bool updated = false;

            comms.setPI(comms.getActiveKp(), comms.getActiveKi());
            comms.setLQRWeights(comms.getActiveK1(), comms.getActiveK2(), comms.getActiveK3());

            if (strcmp(cmd, "state") == 0) { comms.setRobotState(doc["state"], doc["edge"]); updated = true; } 
            else if (strcmp(cmd, "tune_pitch") == 0) { comms.setPitchTarget(doc["val"]); updated = true; } 
            else if (strcmp(cmd, "tune_pi") == 0) { comms.setPI(doc["kp"], doc["ki"]); updated = true; } 
            else if (strcmp(cmd, "tune_lqr") == 0) { comms.setLQRWeights(doc["k1"], doc["k2"], doc["k3"]); updated = true; }

            if (updated) comms.sendCommandPacket();
        }
    }
}

void setup() {
    Serial.begin(115200);      
    comms.begin(500000);    

    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed!");
    } else {
        Serial.println("LittleFS Mounted. Files on chip:");
        File root = LittleFS.open("/");
        File file = root.openNextFile();
        if (!file) Serial.println("  [EMPTY FILESYSTEM]");
        while(file){
            Serial.print("  - /");
            Serial.println(file.name());
            file = root.openNextFile();
        }
    }
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed!");
    }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nDashboard IP: "); 
    Serial.println(WiFi.localIP());

    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html").setCacheControl("max-age=600");
    
    ws.onEvent(onEvent);
    server.addHandler(&ws);
    server.begin();
    Serial.println("Async Server and WebSockets Started!");
}

void loop() {
    ws.cleanupClients();   
    comms.update();        

    // Slow down slightly to 80ms (12.5 Hz) to let the Wi-Fi radio clear its transmit buffers
    if (millis() - lastTelemetryTime > 80) {
        lastTelemetryTime = millis();
        
        // Only do the heavy JSON math and send packets if a browser is actively watching
        if (ws.count() > 0) {
            StaticJsonDocument<1024> doc; 
            doc["p"] = comms.getPitch(); doc["pr"] = comms.getPitchRate(); doc["f"] = comms.getFaultCode();
            doc["sync_kp"] = comms.getActiveKp(); doc["sync_ki"] = comms.getActiveKi();
            doc["sync_k1"] = comms.getActiveK1(); doc["sync_k2"] = comms.getActiveK2(); doc["sync_k3"] = comms.getActiveK3();
            JsonArray q = doc.createNestedArray("q");
            q.add(comms.getQ0()); 
            q.add(comms.getQ1()); 
            q.add(comms.getQ2()); 
            q.add(comms.getQ3());

            JsonArray mv = doc.createNestedArray("mv");
            mv.add(comms.getMotorVel(1)); mv.add(comms.getMotorVel(2)); mv.add(comms.getMotorVel(3));
            JsonArray mc = doc.createNestedArray("mc");
            mc.add(comms.getMotorCur(1)); mc.add(comms.getMotorCur(2)); mc.add(comms.getMotorCur(3));
            JsonArray a = doc.createNestedArray("a");
            a.add(comms.getIMU('x', 'a')); a.add(comms.getIMU('y', 'a')); a.add(comms.getIMU('z', 'a'));
            JsonArray g = doc.createNestedArray("g");
            g.add(comms.getIMU('x', 'g')); g.add(comms.getIMU('y', 'g')); g.add(comms.getIMU('z', 'g'));

            char buffer[512]; 
            size_t len = serializeJson(doc, buffer);
            ws.textAll(buffer, len);
        }
    }

    if (millis() - lastHeartbeat >= 50) {
        lastHeartbeat = millis();
        comms.sendCommandPacket(); 
    }
    
    delay(2); // let the background web tasks breathe
}