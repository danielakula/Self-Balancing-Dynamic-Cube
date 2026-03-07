#include "NetworkManager.h"

NetworkManager::NetworkManager(const char* ssid, const char* password, const char* hostname)
    : _ssid(ssid), _password(password), _hostname(hostname), _isUpdating(false), _otaInitialized(false) {}

void NetworkManager::begin() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(_ssid, _password);
    // We DO NOT wait here. The ESP32 will connect on its own time.
}

void NetworkManager::update() {
    // 1. Check if we just connected and need to start OTA
    if (WiFi.status() == WL_CONNECTED && !_otaInitialized) {
        ArduinoOTA.setHostname(_hostname);
        
        // --- OTA SAFETY HOOK ---
        ArduinoOTA.onStart([this]() {
            Serial.println("\n[OTA] Update Starting! Halting motors...");
            this->_isUpdating.store(true); // Flag the system to shut down!
        });

        ArduinoOTA.onEnd([]() { Serial.println("\n[OTA] Update Complete."); });
        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        });

        ArduinoOTA.begin();
        _otaInitialized = true;
        Serial.print("\n[WIFI] Connected! IP: ");
        Serial.println(WiFi.localIP());
    }

    // 2. Handle OTA (Only if WiFi is connected)
    if (_otaInitialized) {
        ArduinoOTA.handle();
    }
}

bool NetworkManager::isConnected() { return WiFi.status() == WL_CONNECTED; }
bool NetworkManager::isUpdating() { return _isUpdating.load(); }