#include <Arduino.h>

// Pin Definitions
#define KILL_OUT_PIN 6
#define KILL_IN_PIN  15
#define LED_PIN      43  

// Variable to keep track of the last known state
int lastSwitchState = -1; 

void setup() {
    Serial.begin(115200);
    delay(500); 

    // Setup the 3.3V supply pin
    pinMode(KILL_OUT_PIN, OUTPUT);
    digitalWrite(KILL_OUT_PIN, HIGH);

    // Setup the sensing pin with the internal pulldown resistor
    pinMode(KILL_IN_PIN, INPUT_PULLDOWN);

    // Setup the LED indicator
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println("--- Kill Switch Hardware Test ---");
    Serial.println("Flip your switch to test...");
}

void loop() {
    // Read the current state of the switch
    int currentSwitchState = digitalRead(KILL_IN_PIN);

    // If the state has changed since the last time checked...
    if (currentSwitchState != lastSwitchState) {
        
        if (currentSwitchState == HIGH) {
            // Switch is closed (3.3V is flowing from GPIO 6 to GPIO 15)
            Serial.println("STATUS: [ON]  - System LIVE");
            digitalWrite(LED_PIN, HIGH);
        } else {
            // Switch is open (GPIO 15 is pulled LOW by internal resistor)
            Serial.println("STATUS: [OFF] - EMERGENCY HALT");
            digitalWrite(LED_PIN, LOW);
        }
        
        // Update the last state
        lastSwitchState = currentSwitchState;
    }

    // A tiny 50ms delay so physical switch vibrations don't trigger multiple rapid ON/OFF prints
    delay(50); 
}