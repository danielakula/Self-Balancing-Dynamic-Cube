#include <Arduino.h>

// --- Configuration ---
const int PWM_PIN = 17;
const int PWM_FREQ = 10000; // 10kHz is a standard starting point for choppers
const int PWM_RES = 8;      // 8-bit resolution (0 to 255)

int currentDutyCycle = 0;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  delay(1000); // Brief pause to let the terminal connect

  // Initialize the PWM Pin (ESP32 Arduino Core v3.x API)
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
  
  // Start safe: 0% duty cycle (OFF)
  ledcWrite(PWM_PIN, 0); 

  Serial.println("\n--- Brake Chopper PWM Test ---");
  Serial.println("WARNING: Ensure power supply current limit is set safely.");
  Serial.println("Enter target duty cycle percentage (0 - 100):");
}

void loop() {
  // Check if the user has typed a new percentage into the Serial Monitor
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove whitespace/newlines
    
    if (input.length() > 0) {
      int percent = input.toInt();
      
      // Clamp the input for safety (0% to 100%)
      if (percent < 0) percent = 0;
      if (percent > 100) percent = 100;
      
      // Map the percentage (0-100) to our 8-bit resolution (0-255)
      currentDutyCycle = map(percent, 0, 100, 0, 255);
      
      // Apply the new PWM duty cycle
      ledcWrite(PWM_PIN, currentDutyCycle);
      
      // Print confirmation
      Serial.print("Braking Duty Cycle set to: ");
      Serial.print(percent);
      Serial.print("%  (Raw 8-bit value: ");
      Serial.print(currentDutyCycle);
      Serial.println("/255)");
    }
  }
}