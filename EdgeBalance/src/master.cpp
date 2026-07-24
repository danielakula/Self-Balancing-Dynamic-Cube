#include <Arduino.h>
#include <Config.h>
#include "Driver.h"
#include "VoltageMonitor.h"
#include "IMU.h"
#include "LQR.h"
#include "InterchipComms.h"
#include <atomic>

// Define a custom fault code for the web dashboard Disable button
#define FAULT_USER_DISABLE 0x80000000 

// --- Active Parameters ---
float lqr_k1 = DEFAULT_LQR_K1;
float lqr_k2 = DEFAULT_LQR_K2;
float lqr_k3 = DEFAULT_LQR_K3;

float Kp_outer = DEFAULT_KP_OUTER; 
float Ki_outer = DEFAULT_KI_OUTER;

// --- Initialisation ---
float target = 0.0f;
float target_angle = 0.0f;
float wheel_velocity_integral = 0.0f;

// --- Thread-safe mailboxes between Core 0 and Core 1 ---
std::atomic<float> shared_target_torque(0.0f);
std::atomic<float> shared_motor_velocity(0.0f);
std::atomic<float> shared_motor_iq(0.0f);

volatile std::atomic<bool> spiRequested(false);
volatile std::atomic<bool> spiSafeToUse(false);

// --- Global Objects ---
SPIClass hwSpi(1);
HardwareSerial uartLink(1); 
Driver motor1(Motor1Pins, MotorTuning); 
VoltageMonitor battery(VSENSE_PIN, VSENSE_DIVIDER_RATIO, VSENSE_TRIM);
InterchipComms comms(uartLink, MASTER_RX, MASTER_TX); 
IMU_Sensor imu(ISM_CS, &hwSpi);
LQR lqr;

// =======================================================
// CORE 1: MOTOR CONTROL TASK (Fast Loop)
// =======================================================
void taskFOC(void * pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(500));
    disableCore1WDT(); 
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

    for(;;) {
        if (spiRequested.load()) {
            spiSafeToUse.store(true); 
            while (spiRequested.load()) { /* Spin and wait */ }
            spiSafeToUse.store(false); 
        } else {
            motor1.setTarget(shared_target_torque.load());
            motor1.runFOC(); 
            shared_motor_velocity.store(motor1.getVelocity());
            shared_motor_iq.store(motor1.getCurrentQ());
        }
    }
}

// =======================================================
// CORE 0: LQR & TELEMETRY TASK (200Hz)
// =======================================================
void taskLQR(void *pvParameters) {
    uint32_t currentFaults = FAULT_NONE;
    static uint32_t lastFaults = 0xFFFFFFFF; 
    static uint32_t lastPrintTime = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TASK_LOOP_FREQUENCY); // 200 Hz
    const float dt = TASK_LOOP_FREQUENCY / 1000.0f; // Calculate dt strictly from config

    for(;;) {
        currentFaults = FAULT_NONE; 

        // --- 1. FAULT MONITORING ---
        if (battery.isUnderVoltage(BATTERY_SAFE_MIN)) currentFaults |= FAULT_UNDER_VOLTAGE;
        if (battery.isOverVoltage(BATTERY_SAFE_MAX))  currentFaults |= FAULT_OVER_VOLTAGE;
        
        comms.update(); 
        if (!comms.isConnectionAlive(100)) currentFaults |= FAULT_COMMS_LOST;
        if (comms.getRobotState() == 0) currentFaults |= FAULT_USER_DISABLE;

        // --- 2. APPLY INCOMING TUNING PARAMETERS ---
        // PI Tuning
        if (comms.getKpOuter() > 0.00001f) Kp_outer = comms.getKpOuter();
        if (comms.getKiOuter() > 0.00001f) Ki_outer = comms.getKiOuter();
        
        // LQR Tuning (Ignore exact 0.0f on boot)
        if (comms.getK1() != 0.0f) { 
            lqr_k1 = comms.getK1();
            lqr_k2 = comms.getK2();
            lqr_k3 = comms.getK3();
            lqr.setGains(lqr_k1, lqr_k2, lqr_k3);
        }

        // IMU Tuning
        if (comms.getBaseAlpha() > 0.0f) imu.setAlpha(comms.getBaseAlpha());
        if (comms.getAccelTol() > 0.0f) imu.setAccelTolerance(comms.getAccelTol());

        // --- 3. BULLETPROOF SPI BUS ACCESS ---
        spiRequested.store(true); 
        while (!spiSafeToUse.load()) { delayMicroseconds(1); }

        if (motor1.hasHardwareFault()) currentFaults |= FAULT_DRV1;
        currentFaults |= comms.getFaultCode(); 
        
        // State Machine execution during safe SPI window
        if (currentFaults != lastFaults) {
            if (currentFaults != FAULT_NONE) {
                motor1.emergencyStop();
                Serial.printf("\n[SYSTEM HALTED] Mask: 0x%08X\n", currentFaults);
            } else {
                motor1.enable();
            }
            lastFaults = currentFaults;
        }

        imu.update(); // Calculate internal kinematics
        spiRequested.store(false); // RELEASE SPI BUS!
        
        float current_wheel_vel = shared_motor_velocity.load();
        
        // --- 4. OUTER LOOP (PI) ---
        if (currentFaults == FAULT_NONE && abs(imu.getPitch()) < ANGLE_THRESHOLD) {
            wheel_velocity_integral += current_wheel_vel * dt; 
            
            if(wheel_velocity_integral > INTEGRAL_CLAMPING) wheel_velocity_integral = INTEGRAL_CLAMPING;
            if(wheel_velocity_integral < -INTEGRAL_CLAMPING) wheel_velocity_integral = -INTEGRAL_CLAMPING;

            // Calculate PI target, PLUS the manual target pitch from the Web Dashboard slider!
            target_angle = (Kp_outer * current_wheel_vel) + (Ki_outer * wheel_velocity_integral) + comms.getTargetPitch();
            
            if (target_angle > MAX_PITCH_TARGET) target_angle = MAX_PITCH_TARGET;
            if (target_angle < -MAX_PITCH_TARGET) target_angle = -MAX_PITCH_TARGET;

        } else {
            target_angle = 0.0f;
            wheel_velocity_integral = 0.0f;
        }

        // --- 5. INNER LOOP (LQR) ---
        if (currentFaults == FAULT_NONE) {
            if (abs(imu.getPitch()) > ANGLE_THRESHOLD) {
                target = 0.0f; 
            } else {
                float theta_error = imu.getPitch() - target_angle;
                
                target = -lqr.compute(
                    theta_error,            // x1: Angle ERROR
                    imu.getPitchRate(),     // x2: Cube Velocity
                    current_wheel_vel,      // x3: Wheel Velocity
                    dt                      // dt: Fixed time step
                );
            }
        } else {
            target = 0.0f;
        }

        shared_target_torque.store(target);

        // --- 6. SEND TELEMETRY TO WI-FI MCU ---
        comms.setTelemetryTuning(Kp_outer, Ki_outer, lqr_k1, lqr_k2, lqr_k3);
        comms.setTelemetryMotors(current_wheel_vel, 0, 0, shared_motor_iq.load(), 0, 0);
        comms.setTelemetryKinematics(imu.getPitch() * RAD_TO_DEG, imu.getPitchRate() * RAD_TO_DEG, currentFaults);
        
        // Include the Mahony raw data stream!
        comms.setTelemetryIMU(imu.getAccelX(), imu.getAccelY(), imu.getAccelZ(), 
                              imu.getGyroX(), imu.getGyroY(), imu.getGyroZ());
        
        comms.sendTelemetryPacket();

        // --- 7. DEBUG OUTPUT ---
        if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
            lastPrintTime = millis();
            Serial.printf("Angle:%.2f, TargetAngle:%.2f, WheelVel:%.2f, Kp:%.4f, Faults:0x%X\n", 
                          imu.getPitch()*RAD_TO_DEG, target_angle*RAD_TO_DEG, current_wheel_vel, Kp_outer, currentFaults);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
    Serial.begin(115200);
    comms.begin(1000000); 

    delay(2000); 
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

    xTaskCreatePinnedToCore(taskFOC, "MotorTask", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(taskLQR, "LQR_Task", 8192, NULL, 4, NULL, 0);
}

void loop() {
    vTaskDelete(NULL); 
}