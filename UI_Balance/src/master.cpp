#include <Arduino.h>
#include <Config.h>
#include "Driver.h"
#include "VoltageMonitor.h"
#include "IMU.h"
#include "LQR.h"
#include "InterchipComms.h"
#include <atomic>
#include <esp_task_wdt.h>
#include <MahonyAHRS.h> 

Mahony filter;

// Thread-safe mailboxes between Core 0 and Core 1
std::atomic<float> shared_target_torque(0.0f);
std::atomic<float> shared_motor_velocity(0.0f);
std::atomic<float> shared_motor_iq(0.0f);

volatile std::atomic<bool> spiRequested(false);
volatile std::atomic<bool> spiSafeToUse(false);

SPIClass hwSpi(1);
HardwareSerial uartLink(1); 
Driver motor1(Motor1Pins, MotorTuning); 
VoltageMonitor battery(VSENSE_PIN, VSENSE_DIVIDER_RATIO, VSENSE_TRIM);
InterchipComms comms(uartLink, MASTER_RX, MASTER_TX); 
IMU_Sensor imu(ISM_CS, &hwSpi);
LQR lqr;

float target = 0.0f;
float target_angle = 0.0f;
float wheel_velocity_integral = 0.0f;

float Kp_outer = 0.003f; //0.003f; 
float Ki_outer = 0.0001f; //0.0001f;

unsigned long bootTime = 0;

void taskFOC(void * pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(500));
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

    for(;;) {
        // EXACT SPINLOCK FROM OLD CODE
        if (spiRequested.load()) {
            spiSafeToUse.store(true); 
            while (spiRequested.load()) { 
            }
            spiSafeToUse.store(false); 
        } else {
            motor1.setTarget(shared_target_torque.load());
            motor1.runFOC(); 
            shared_motor_velocity.store(motor1.getVelocity());
            shared_motor_iq.store(motor1.getCurrentQ());
        }
    }
}

void taskLQR(void *pvParameters) {
    // =======================================================
    // INITIALIZE COMMS ON CORE 0 TO PROTECT MOTOR INTERRUPTS
    // =======================================================
    uartLink.setTxBufferSize(1024);
    comms.begin(500000);
    bootTime = millis();

    uint32_t currentFaults = FAULT_NONE;
    static uint32_t lastFaults = 0xFFFFFFFF; 

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // EXACT 200Hz Loop
    const float dt = 0.005f; 

    for(;;) {
        currentFaults = FAULT_NONE; 

        if (battery.isUnderVoltage(BATTERY_SAFE_MIN)) currentFaults |= FAULT_UNDER_VOLTAGE;
        if (battery.isOverVoltage(BATTERY_SAFE_MAX))  currentFaults |= FAULT_OVER_VOLTAGE;

        comms.update(); 
        
        if (millis() - bootTime > 3000) {
            if (!comms.isConnectionAlive(1000)) currentFaults |= FAULT_COMMS_LOST;
        }

        if (comms.getRobotState() == 0) currentFaults |= FAULT_USER_DISABLE;

        // Live Tuning Updates
        if (comms.getKpOuter() > 0.00001f) Kp_outer = comms.getKpOuter();
        if (comms.getKiOuter() > 0.00001f) Ki_outer = comms.getKiOuter();
        if (comms.getK1() != 0.0f) lqr.setGains(comms.getK1(), comms.getK2(), comms.getK3());

        // =======================================================
        // EXACT SPI BUS ACCESS FROM OLD CODE
        // =======================================================
        spiRequested.store(true); 
        while (!spiSafeToUse.load()) { delayMicroseconds(1); }

        if (motor1.hasHardwareFault()) currentFaults |= FAULT_DRV1;
        
        // NO SERIAL PRINTING 
        if (currentFaults != lastFaults) {
            if (currentFaults != FAULT_NONE) motor1.emergencyStop();
            else motor1.enable();
        }

        imu.update(); 

        spiRequested.store(false);
        
        if (currentFaults != lastFaults) {
            if (currentFaults != FAULT_NONE) {
                Serial.printf("\n[SYSTEM HALTED] Mask: 0x%08X\n", currentFaults);
            } else {
                Serial.println("\n[SYSTEM ACTIVE] Balancing...");
            }
            lastFaults = currentFaults;
        }

        float current_wheel_vel = shared_motor_velocity.load();
        float current_iq = shared_motor_iq.load();
        
        // =======================================================
        // EXACT OUTER LOOP MATH FROM OLD CODE
        // =======================================================
        if (currentFaults == FAULT_NONE && abs(imu.getPitch()) < 0.35f) {
            wheel_velocity_integral += current_wheel_vel * dt; 
            
            if(wheel_velocity_integral > 500.0f) wheel_velocity_integral = 500.0f;
            if(wheel_velocity_integral < -500.0f) wheel_velocity_integral = -500.0f;

            target_angle = (Kp_outer * current_wheel_vel) + (Ki_outer * wheel_velocity_integral);
            
            const float MAX_TILT = 0.052f; 
            if (target_angle > MAX_TILT) target_angle = MAX_TILT;
            if (target_angle < -MAX_TILT) target_angle = -MAX_TILT;
        } else {
            target_angle = 0.0f;
            wheel_velocity_integral = 0.0f;
        }

        // =======================================================
        // EXACT INNER LOOP MATH FROM OLD CODE
        // =======================================================
        if (currentFaults == FAULT_NONE) {
            if (abs(imu.getPitch()) > 0.35f) {
                target = 0.0f; 
            } else {
                float theta_error = imu.getPitch() - target_angle;
                target = -lqr.compute(theta_error, imu.getPitchRate(), current_wheel_vel, dt);
            }
        } else { target = 0.0f; }

        shared_target_torque.store(target);

        // =======================================================
        // BACKGROUND TELEMETRY 
        // =======================================================
        filter.update(imu.getGyroX(), imu.getGyroY(), imu.getGyroZ(), 
                      imu.getAccelX(), imu.getAccelY(), imu.getAccelZ(), dt);

        comms.setTelemetryQuat(filter.q0, filter.q1, filter.q2, filter.q3);
        comms.setTelemetryTuning(Kp_outer, Ki_outer, 0, 0, 0); 
        comms.setTelemetryMotors(current_wheel_vel, 0, 0, current_iq, 0, 0);
        comms.setTelemetryKinematics(imu.getPitch() * RAD_TO_DEG, imu.getPitchRate() * RAD_TO_DEG, currentFaults);
        comms.setTelemetryIMU(imu.getAccelX(), imu.getAccelY(), imu.getAccelZ(), imu.getGyroX(), imu.getGyroY(), imu.getGyroZ());
        
        static uint32_t lastTelemetrySend = 0;
        if (millis() - lastTelemetrySend > 40) { 
            lastTelemetrySend = millis();
            comms.sendTelemetryPacket();
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
    Serial.setTxBufferSize(4096);
    Serial.begin(921600);
    delay(2000); 

    Serial.println("\n\n=======================================");
    Serial.println("       BALANCE BOT BOOTING...          ");  
    Serial.println("=======================================\n");

    SimpleFOCDebug::enable(&Serial);
    hwSpi.begin(MASTER_SCK, MASTER_MISO, MASTER_MOSI, -1);
    
    motor1.begin(&hwSpi);
    battery.begin();
    
    if (!imu.init()) {
        Serial.println("IMU init failed!");
        while (1);
    }
    imu.calibrate();

    TaskHandle_t idle_0 = xTaskGetIdleTaskHandleForCPU(0);
    if(idle_0 != NULL) esp_task_wdt_delete(idle_0);

    xTaskCreatePinnedToCore(taskFOC, "MotorTask", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(taskLQR, "LQR_Task", 8192, NULL, 4, NULL, 0);
}

void loop() {
    vTaskDelete(NULL); 
}