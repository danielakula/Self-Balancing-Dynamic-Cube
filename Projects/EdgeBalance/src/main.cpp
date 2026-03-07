#include <Arduino.h>
#include <Config.h>
#include "Driver.h"
#include "VoltageMonitor.h"
#include "IMU.h"
#include "LQR.h"
#include <atomic>

// Thread-safe mailboxes between Core 0 and Core 1
std::atomic<float> shared_target_torque(0.0f);
std::atomic<float> shared_motor_velocity(0.0f);
std::atomic<float> shared_motor_iq(0.0f);

// Make sure your spinlock flags are volatile so the compiler doesn't optimize them away!
volatile std::atomic<bool> spiRequested(false);
volatile std::atomic<bool> spiSafeToUse(false);

SPIClass hwSpi(1);
Driver motor1(Motor1Pins, MotorTuning); 
VoltageMonitor battery(VSENSE_PIN, VSENSE_DIVIDER_RATIO, VSENSE_TRIM);

float target = 0.0f;

// Instantiate globally with your specific SPI pins
IMU_Sensor imu(ISM_CS, &hwSpi);

LQR lqr;

void taskFOC(void * pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(500));
    // 1. Assassinate the Watchdog on Core 1 so we never have to yield!
    disableCore1WDT(); 

    // 2. Set this specific task to the highest possible FreeRTOS priority
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

    for(;;) {
        // 1. The Spinlock Handshake
        if (spiRequested.load()) {
            // Core 0 needs the bus. Grant access.
            spiSafeToUse.store(true); 
            
            // Spin in place and wait for Core 0 to finish reading SPI sensors
            while (spiRequested.load()) { 
                // Do nothing. Just wait.
            }
            
            // Core 0 is done. Revoke access and resume math.
            spiSafeToUse.store(false); 
        } else {
            //motor1.setTarget(0.0f);
            motor1.setTarget(shared_target_torque.load());
            motor1.runFOC(); 
            shared_motor_velocity.store(motor1.getVelocity());
            shared_motor_iq.store(motor1.getCurrentQ());
        }
    }
}

void taskLQR(void *pvParameters) {
    uint32_t currentFaults = FAULT_NONE;
    static uint32_t lastFaults = 0xFFFFFFFF; 
    static uint32_t lastPrintTime = 0;
    const uint32_t PRINT_INTERVAL_MS = 200; 

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200 Hz LQR loop

    for(;;) {
        currentFaults = FAULT_NONE; 

        if (battery.isUnderVoltage(BATTERY_SAFE_MIN)) currentFaults |= FAULT_UNDER_VOLTAGE;
        if (battery.isOverVoltage(BATTERY_SAFE_MAX))  currentFaults |= FAULT_OVER_VOLTAGE;

        // =======================================================
        // BULLETPROOF SPI BUS ACCESS
        // =======================================================
        spiRequested.store(true); // 1. Ask Core 1 for the bus
        
        while (!spiSafeToUse.load()) { 
            // 2. Wait right here until Core 1 finishes its math and grants access
            delayMicroseconds(1); 
        }

        // 3. We now have 100% exclusive, safe access to the SPI hardware!
        if (motor1.hasHardwareFault()) currentFaults |= FAULT_DRV1;
        
        // State Machine (emergencyStop/enable might send SPI commands to the DRV chip)
        if (currentFaults != lastFaults) {
            if (currentFaults != FAULT_NONE) {
                motor1.emergencyStop();
                Serial.printf("\n[SYSTEM HALTED] Mask: 0x%08X\n", currentFaults);
            } else {
                motor1.enable();
            }
            lastFaults = currentFaults;
        }

        imu.update(); // Automatically calculates dt and updates state!

        spiRequested.store(false);
        
        // 2. Safely grab the wheel velocity from Core 0
        float current_wheel_vel = shared_motor_velocity.load();
        
        // 3. Pass states to the LQR Controller
        // Always update target current if healthy
        if (currentFaults == FAULT_NONE) {
          target = -lqr.compute(
          imu.getPitch(),         // x1: Cube Angle
          imu.getPitchRate(),     // x2: Cube Velocity
          current_wheel_vel,      // x3: Wheel Velocity
          0.005f                  // dt: 5ms
          );
        }
        
        if (abs(imu.getPitch()) > 0.35f) {
                target = 0.0f; 
        }

        // 5. Safely send the torque command to Core 0
        shared_target_torque.store(target);

        // 3. Telemetry Monitor
        if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
            lastPrintTime = millis();
            
            // Plotter format: "Label1:Value1,Label2:Value2"
            Serial.printf("Angle:%.2f, PitchRate:%.2f, Target:%.2f, WheelVel:%.2f, Iq:%.2f, Battery:%.2fV\n", 
                          imu.getPitch()*RAD_TO_DEG, 
                          imu.getPitchRate(), target, current_wheel_vel, shared_motor_iq.load(), battery.readVoltage());
        }
        // if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
        //     lastPrintTime = millis();
        //     float safe_iq = shared_motor_iq.load();
        //     Serial.printf("Batt: %05.2fV | Angle: %05.2f deg | Cube Velocity: %05.2f | Wheel Velocity:%05.1f Iq:%05.2f Target:%05.2f\n",
        //                   battery.readVoltage(), imu.getPitch(), imu.getPitchRate(), motor1.getVelocity(), motor1.getCurrentQ(), target);
        // }
        
        // Sleep until exactly 5ms have passed
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000); // Wait for Serial to initialize
    Serial.println("\n\n=======================================");
    Serial.println("       BALANCE BOT BOOTING...          ");  
    Serial.println("=======================================\n");

    SimpleFOCDebug::enable(&Serial);

    hwSpi.begin(MASTER_SCK, MASTER_MISO, MASTER_MOSI, -1);
    
    motor1.begin(&hwSpi);
    battery.begin();

    lqr.setCurrentLimit(MotorTuning.current_limit);
    
    if (!imu.init()) {
        Serial.println("IMU init failed!");
        while (1);
    }
    
    imu.calibrate();

    disableCore0WDT();

    // Pin FOC to Core 0 (Priority 5 - Highest)
    xTaskCreatePinnedToCore(taskFOC, "MotorTask", 8192, NULL, 5, NULL, 1);
    
    // Pin LQR to Core 1 (Priority 4 - High)
    xTaskCreatePinnedToCore(taskLQR, "LQR_Task", 8192, NULL, 4, NULL, 0);
}

void loop() {
    vTaskDelete(NULL); 
}