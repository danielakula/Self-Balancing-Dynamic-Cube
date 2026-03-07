#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <atomic>

class NetworkManager {
public:
    NetworkManager(const char* ssid, const char* password, const char* hostname);
    
    void begin();
    void update(); // Call this in Core 0 loop
    
    bool isConnected();
    bool isUpdating(); // Used to trigger our safety fault

private:
    const char* _ssid;
    const char* _password;
    const char* _hostname;
    
    std::atomic<bool> _isUpdating;
    bool _otaInitialized;
};