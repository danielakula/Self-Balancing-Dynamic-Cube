#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "InterchipComms.h"
#include "Config.h"

// --- Global Objects ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
InterchipComms comms(Serial1, SLAVE_RX, SLAVE_TX);

// --- WebSocket Message Handler ---
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, data, len);
        if (error) {
            Serial.print("JSON Parse Failed: ");
            Serial.println(error.c_str());
            return;
        }

        const char* cmd = doc["cmd"];
        bool updated = false;

        if (strcmp(cmd, "state") == 0) {
            comms.setRobotState(doc["state"], doc["edge"]);
            updated = true;
        } 
        else if (strcmp(cmd, "tune_pitch") == 0) {
            comms.setPitchTarget(doc["val"]);
            updated = true;
        } 
        else if (strcmp(cmd, "tune_pi") == 0) {
            comms.setPI(doc["kp"], doc["ki"]);
            updated = true;
        } 
        else if (strcmp(cmd, "tune_lqr") == 0) {
            comms.setLQRWeights(doc["k1"], doc["k2"], doc["k3"]);
            updated = true;
        }

        if (updated) {
            comms.sendCommandPacket();
            Serial.println("Updated Parameters Sent to Control MCU");
        }
    }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        handleWebSocketMessage(arg, data, len);
    } else if (type == WS_EVT_CONNECT) {
        Serial.println("Phone Connected!");
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.println("Phone Disconnected!");
    }
}

// =======================================================
// CORE 0: COMMS & TELEMETRY TASK
// =======================================================
void taskWiFiTelemetry(void *pvParameters) {
    unsigned long lastTelemetryTime = 0;
    const unsigned long TELEMETRY_INTERVAL = 50; // 20Hz

    for(;;) {
        // 1. Keep parsing incoming packets from the Control MCU
        comms.update();

        // 2. Broadcast Telemetry to the Phone
        if (millis() - lastTelemetryTime > TELEMETRY_INTERVAL) {
            lastTelemetryTime = millis();

            if (ws.count() > 0) {
                StaticJsonDocument<512> doc; 

                // Kinematics & Faults
                doc["p"] = comms.getPitch();
                doc["pr"] = comms.getPitchRate();
                doc["f"] = comms.getFaultCode();

                // Active sync data
                doc["sync_kp"] = comms.getActiveKp();
                doc["sync_ki"] = comms.getActiveKi();
                doc["sync_k1"] = comms.getActiveK1();
                doc["sync_k2"] = comms.getActiveK2();
                doc["sync_k3"] = comms.getActiveK3();

                // Motor Telemetry
                JsonArray mv = doc.createNestedArray("mv");
                mv.add(comms.getMotorVel(1)); mv.add(comms.getMotorVel(2)); mv.add(comms.getMotorVel(3));
                
                JsonArray mc = doc.createNestedArray("mc");
                mc.add(comms.getMotorCur(1)); mc.add(comms.getMotorCur(2)); mc.add(comms.getMotorCur(3));

                // IMU Telemetry
                JsonArray a = doc.createNestedArray("a");
                a.add(comms.getIMU('x', 'a')); a.add(comms.getIMU('y', 'a')); a.add(comms.getIMU('z', 'a'));
                
                JsonArray g = doc.createNestedArray("g");
                g.add(comms.getIMU('x', 'g')); g.add(comms.getIMU('y', 'g')); g.add(comms.getIMU('z', 'g'));

                // Serialize and blast out via WebSockets
                char buffer[512];
                size_t len = serializeJson(doc, buffer);
                ws.textAll(buffer, len);
            }
        }
        
        // Yield to the FreeRTOS scheduler to prevent Watchdog panics on Core 0
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

void setup() {
    Serial.begin(115200);      
    comms.begin(1000000);      

    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Dashboard IP: ");
    Serial.println(WiFi.localIP());

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/index.html", "text/html");
    });

    ws.onEvent(onEvent);
    server.addHandler(&ws);
    server.begin();
    
    // --- PIN THE COMMS TASK TO CORE 0 ---
    // Priority 1 is fine for telemetry. 
    xTaskCreatePinnedToCore(taskWiFiTelemetry, "CommsTask", 8192, NULL, 1, NULL, 0);

    Serial.println("System Ready. Core 1 is now free.");
}

void loop() {
    // Delete the default Arduino loop task, freeing Core 1 completely!
    vTaskDelete(NULL); 
}