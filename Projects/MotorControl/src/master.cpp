#include <Arduino.h>
#include "Config.h"
#include "Driver.h"
#include "VoltageMonitor.h"
#include "InterchipComms.h"

// Hardware Objects
SPIClass hwSpi(1);
HardwareSerial uartLink(1); 

// Functional Objects
Driver motor1(Motor1Pins, MotorTuning); 
VoltageMonitor battery(VSENSE_PIN, VSENSE_DIVIDER_RATIO, VSENSE_TRIM);
InterchipComms comms(uartLink, MASTER_RX, MASTER_TX); 

float userTargetCurrent = 0.0f;

// =========================================================
// THE SPINLOCK HANDSHAKE FLAGS
// =========================================================
volatile bool spiRequested = false; // Core 0 raises this to ask for the bus
volatile bool spiSafeToUse = false; // Core 1 raises this to grant permission

// =========================================================
// CORE 0: SAFETY, TELEMETRY, & COMMS (Runs at 200Hz)
// =========================================================
void commsAndSafetyTask(void * pvParameters) {
    uint32_t currentFaults = FAULT_NONE;
    static uint32_t lastFaults = 0xFFFFFFFF; 
    static uint32_t lastPrintTime = 0;
    const uint32_t PRINT_INTERVAL_MS = 200; 

    for(;;) {
        currentFaults = FAULT_NONE; 

        comms.update(); 
        if (Serial.available()) {
            userTargetCurrent = Serial.parseFloat(); 
            while(Serial.available()) { Serial.read(); } 
        }

        // 1. Hardware Checks
        if (battery.isUnderVoltage(BATTERY_SAFE_MIN)) currentFaults |= FAULT_UNDER_VOLTAGE;
        if (battery.isOverVoltage(BATTERY_SAFE_MAX))  currentFaults |= FAULT_OVER_VOLTAGE;
        if (!comms.isConnectionAlive(15))             currentFaults |= FAULT_COMMS_LOST;
        
        // =======================================================
        // BULLETPROOF SPI BUS ACCESS
        // =======================================================
        spiRequested = true; // 1. Ask Core 1 for the bus
        
        while (!spiSafeToUse) { 
            // 2. Wait right here until Core 1 finishes its math and grants access
            delayMicroseconds(1); 
        }

        // 3. We now have 100% exclusive, safe access to the SPI hardware!
        if (motor1.hasHardwareFault()) currentFaults |= FAULT_DRV1;
        
        spiRequested = false; // 4. Release the bus so Core 1 can resume
        // =======================================================

        currentFaults |= comms.getRemoteFaultCode(); 

        // 2. STATE MACHINE
        if (currentFaults != lastFaults) {
            if (currentFaults != FAULT_NONE) {
                motor1.emergencyStop();
                Serial.printf("\n[SYSTEM HALTED] Mask: 0x%08X\n", currentFaults);
            } else {
                motor1.enable();
            }
            lastFaults = currentFaults;
        }

        // Always update target current if healthy
        if (currentFaults == FAULT_NONE) {
            motor1.setTarget(userTargetCurrent);
        }

        comms.sendPacket(userTargetCurrent, currentFaults);

        // 3. Telemetry Monitor
        if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
            lastPrintTime = millis();
            Serial.printf("Batt: %05.2fV | Tgt: %05.2fA | M1(V:%05.1f Iq:%05.2f) | M2(V:%05.1f Iq:%05.2f) | M3(V:%05.1f Iq:%05.2f)\n",
                          battery.readVoltage(), userTargetCurrent, 
                          motor1.getVelocity(), motor1.getCurrentQ(), 
                          comms.getMotor2Velocity(), comms.getMotor2Current(), 
                          comms.getMotor3Velocity(), comms.getMotor3Current());
        }
        
        vTaskDelay(pdMS_TO_TICKS(5)); 
    }
}

// =========================================================
// CORE 1: PURE MATH & FOC (UNTHROTTLED)
// =========================================================
void motorTask(void * pvParameters) {
    // 1. Assassinate the Watchdog on Core 1 so we never have to yield!
    disableCore1WDT(); 

    // 2. Set this specific task to the highest possible FreeRTOS priority
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

    for(;;) {
        // 3. The Spinlock Handshake
        if (spiRequested) {
            // Core 0 needs the bus. Grant access.
            spiSafeToUse = true; 
            
            // Spin in place and wait for Core 0 to finish reading faults
            while (spiRequested) { 
                // Do nothing. Just wait.
            }
            
            // Core 0 is done. Revoke access and resume math.
            spiSafeToUse = false; 
        } else {
            // Bus is ours! Run the high-speed FOC math!
            motor1.runFOC(); 
        }
    }
}

// =========================================================
// SETUP & OS BOOT
// =========================================================
void setup() {
    Serial.begin(115200);
    delay(3000); 
    Serial.println("\n\n=======================================");
    Serial.println("       MASTER MCU BOOTING...           ");
    Serial.println("=======================================\n");

    SimpleFOCDebug::enable(&Serial);

    // Start SPI at our safe, tested 2MHz frequency
    hwSpi.begin(MASTER_SCK, MASTER_MISO, MASTER_MOSI, -1);
    
    motor1.begin(&hwSpi);
    battery.begin();
    comms.begin(1000000); 

    Serial.println("\nWaiting for Slave MCU to finish alignment...");
    
    while (!comms.isConnectionAlive(1000)) {
        comms.update(); 
        delay(5);       
    }
    
    Serial.println("Slave MCU Connected! System is synchronized and READY.");

    // Launch Core 0 Task
    xTaskCreatePinnedToCore(commsAndSafetyTask, "CommsLoop", 10000, NULL, 2, NULL, 0);

    // Launch Core 1 High-Speed Task
    xTaskCreatePinnedToCore(motorTask, "MotorLoop", 10000, NULL, 5, NULL, 1);
}

void loop() {
    vTaskDelete(NULL); 
}