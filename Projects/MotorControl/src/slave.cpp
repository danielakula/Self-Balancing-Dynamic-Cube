#include <Arduino.h>
#include "Config.h"
#include "Driver.h"
#include "InterchipComms.h"

// Hardware Objects
SPIClass hwSpi(1);
HardwareSerial uartLink(1);

// Functional Objects 
// (Constructors are now mutex-free)
Driver motor2(Motor2Pins, MotorTuning);
Driver motor3(Motor3Pins, MotorTuning);
InterchipComms comms(uartLink, SLAVE_RX, SLAVE_TX);

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

    for(;;) {
        currentFaults = FAULT_NONE;

        // 1. Process Incoming Data from Master
        comms.update();
        float targetIq = comms.getTargetCurrent();
        uint32_t masterFaults = comms.getRemoteFaultCode();

        // =======================================================
        // BULLETPROOF SPI BUS ACCESS (Shared by Motor 2 & 3)
        // =======================================================
        spiRequested = true; 
        
        while (!spiSafeToUse) { 
            delayMicroseconds(1); 
        }

        // We now have exclusive access to the hardware bus
        if (motor2.hasHardwareFault()) currentFaults |= FAULT_DRV2;
        if (motor3.hasHardwareFault()) currentFaults |= FAULT_DRV3;
        
        spiRequested = false; 
        // =======================================================

        uint32_t systemFaults = currentFaults | masterFaults;

        // 2. SLAVE STATE MACHINE
        if (systemFaults != lastFaults) {
            if (systemFaults != FAULT_NONE) {
                motor2.emergencyStop();
                motor3.emergencyStop();
                Serial.printf("\n[SLAVE HALTED] Mask: 0x%08X\n", systemFaults);
            } else {
                motor2.enable();
                motor3.enable();
            }
            lastFaults = systemFaults;
        }

        // 3. Update Targets if healthy
        if (systemFaults == FAULT_NONE) {
            motor2.setTarget(targetIq);
            motor3.setTarget(targetIq);
        }

        // 4. Send Telemetry back to Master
        comms.setTelemetry(motor2.getVelocity(), motor3.getVelocity(), 
                           motor2.getCurrentQ(), motor3.getCurrentQ());
        comms.sendPacket(0.0f, currentFaults);

        vTaskDelay(pdMS_TO_TICKS(5)); 
    }
}

// =========================================================
// CORE 1: PURE MATH & DUAL FOC (UNTHROTTLED)
// =========================================================
void motorTask(void * pvParameters) {
    // 1. Kill the Watchdog on Core 1
    disableCore1WDT(); 

    // 2. Set to highest priority
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
    
    for(;;) { 
        // 3. The Spinlock Handshake
        if (spiRequested) {
            spiSafeToUse = true; 
            while (spiRequested) { /* Wait for Core 0 */ }
            spiSafeToUse = false; 
        } else {
            // Bus is ours! Run both motors sequentially.
            // Note: Sequential execution is required because they share the MISO/MOSI lines.
            motor2.runFOC(); 
            motor3.runFOC(); 
        }
    }
}

// =========================================================
// SETUP
// =========================================================
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n\n=======================================");
    Serial.println("       SLAVE MCU BOOTING...            ");
    Serial.println("=======================================\n");

    SimpleFOCDebug::enable(&Serial);

    // Standard safe SPI speed for internal bus
    hwSpi.begin(SLAVE_SCK, SLAVE_MISO, SLAVE_MOSI, -1);
    
    motor2.begin(&hwSpi);
    motor3.begin(&hwSpi);
    comms.begin(1000000);
    
    // Launch Tasks
    xTaskCreatePinnedToCore(commsAndSafetyTask, "CommsLoop", 10000, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(motorTask, "MotorLoop", 10000, NULL, 5, NULL, 1);
}

void loop() {
    vTaskDelete(NULL); 
}