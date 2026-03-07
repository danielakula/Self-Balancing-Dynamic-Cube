#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>
#include <U8g2lib.h>
#include "esp_task_wdt.h"

TaskHandle_t MotorTaskHandle;

SemaphoreHandle_t spiMutex = xSemaphoreCreateMutex();

// ================================================================
//  PIN DEFINITIONS
// ================================================================
#define MASTER_SCK   37
#define MASTER_MISO  39
#define MASTER_MOSI  38
#define ENC1_CS      36
#define DRV_CS       48
#define DRV_EN       35
#define nFAULT       45

#define AH_PIN  47
#define AL_PIN  21
#define BH_PIN  14
#define BL_PIN  13
#define CH_PIN  12
#define CL_PIN  11

#define SOA_PIN  3
#define SOB_PIN  9
#define SOC_PIN  10

#define VSENSE_PIN 8      // GPIO 8 (ADC1_CH7)
#define SAMPLES 64        
const float dividerRatio = 11.0; 
const float calibrationTrim = 1.008338;

#define MASTER_SDA 4
#define MASTER_SCL 5

// Initialize the display (Adjust the constructor to match your specific OLED, e.g., SSD1306)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE, MASTER_SCL, MASTER_SDA);

// Global variables to store data for the display
float display_V = 0;
float display_I = 0;

// ================================================================
//  CURRENT SENSE PARAMETERS
//  R_shunt  = 10 mOhm = 0.010 Ohm
//  CSA gain = 20 V/V  (Reg 0x06 bits 7:6 = 10b -> 20V/V)
// ================================================================
#define R_SHUNT     0.010f   // 10 mOhm
#define CSA_GAIN    20.0f    // 20 V/V
#define I_LIMIT     5.0f     // Amps

// ================================================================
//  SPI — single bus, manual CS
// ================================================================
SPIClass hwSpi(1);

// ================================================================
//  DRV8323S
// ================================================================
void drvWriteSpi(uint8_t address, uint16_t data) {
    // Take the mutex - wait up to 100ms if someone else has it
    if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        digitalWrite(ENC1_CS, HIGH);
        uint16_t frame = ((uint16_t)(address & 0x0F) << 11) | (data & 0x7FF);
        hwSpi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
        digitalWrite(DRV_CS, LOW);
        delayMicroseconds(1);
        hwSpi.transfer16(frame);
        delayMicroseconds(1);
        digitalWrite(DRV_CS, HIGH);
        hwSpi.endTransaction();
        
        xSemaphoreGive(spiMutex); // Release the bus
    }
}

uint16_t drvReadSpi(uint8_t address) {
    uint16_t result = 0;
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
        digitalWrite(ENC1_CS, HIGH);
        uint16_t frame = (1 << 15) | ((uint16_t)(address & 0x0F) << 11);
        hwSpi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
        digitalWrite(DRV_CS, LOW);
        delayMicroseconds(1);
        result = hwSpi.transfer16(frame);
        delayMicroseconds(1);
        digitalWrite(DRV_CS, HIGH);
        hwSpi.endTransaction();
        xSemaphoreGive(spiMutex);
    }
    return result & 0x7FF;
}

void initDRV8323S() {
    Serial.println("[DRV] Configuring...");
    digitalWrite(DRV_EN, LOW);  delay(50);
    digitalWrite(DRV_EN, HIGH); delay(100);

    drvWriteSpi(0x02, 0b00000000000);
    drvWriteSpi(0x03, 0b01100100010);
    drvWriteSpi(0x04, 0b10100100010);
    drvWriteSpi(0x05, 0b01000011001);
    drvWriteSpi(0x06, 0b01110000011);

    Serial.printf("[DRV] Reg02: 0x%03X\n", drvReadSpi(0x02));
    Serial.printf("[DRV] Reg03: 0x%03X\n", drvReadSpi(0x03));
    Serial.printf("[DRV] Reg04: 0x%03X\n", drvReadSpi(0x04));
    Serial.printf("[DRV] Reg05: 0x%03X\n", drvReadSpi(0x05));
    Serial.printf("[DRV] Reg06: 0x%03X\n", drvReadSpi(0x06));
    Serial.printf("[DRV] Fault1: 0x%03X  Fault2: 0x%03X\n",
                  drvReadSpi(0x00), drvReadSpi(0x01));
}

// ================================================================
//  SIMPLEFOC OBJECTS
// ================================================================
BLDCDriver6PWM driver = BLDCDriver6PWM(AH_PIN, AL_PIN, BH_PIN, BL_PIN, CH_PIN, CL_PIN);
BLDCMotor      motor  = BLDCMotor(14, 0.5f, 210.0f, 0.00018f);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, ENC1_CS);
LowsideCurrentSense currentSense = LowsideCurrentSense(R_SHUNT, CSA_GAIN, SOA_PIN, SOB_PIN, SOC_PIN);

Commander command = Commander(Serial);
float target = 0.0f;  // Target current in Amps

void onTarget(char* cmd) { command.scalar(&target, cmd); }

// ================================================================
//  THE REAL-TIME MOTOR TASK
// ================================================================
void motorRuntime(void * pvParameters) {
    // 1. Tell the Arduino ESP32 Core to disable the Watchdog on Core 0 entirely
    disableCore0WDT(); 
    
    // 2. Fallback: Specifically unsubscribe the Core 0 IDLE task from the TWDT
    // This is required on some newer ESP32 board manager versions.
    TaskHandle_t idleTaskHandle = xTaskGetIdleTaskHandleForCPU(0);
    if (idleTaskHandle != NULL) {
        esp_task_wdt_delete(idleTaskHandle);
    }

    // 3. Set to Maximum Priority
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

    for(;;) {
        // 4. Secure the SPI bus
        if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
            motor.loopFOC();
            motor.move(target);
            xSemaphoreGive(spiMutex);
        }
        
        // NO DELAY. NO YIELD.
        // We let this loop spin as fast as the silicon allows.
    }
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== GL40 TORQUE CONTROL — LOW SIDE CURRENT SENSE ===");

    // Pins
    pinMode(DRV_CS,  OUTPUT); digitalWrite(DRV_CS,  HIGH);
    pinMode(ENC1_CS, OUTPUT); digitalWrite(ENC1_CS, HIGH);
    pinMode(DRV_EN,  OUTPUT); digitalWrite(DRV_EN,  HIGH);
    pinMode(nFAULT,  INPUT_PULLUP);

    // SPI
    hwSpi.begin(MASTER_SCK, MASTER_MISO, MASTER_MOSI, -1);

    // Display
    u8g2.begin();
    u8g2.setFont(u8g2_font_7x14_tr);

    // DRV
    initDRV8323S();

    // Encoder
    sensor.init(&hwSpi);
    Serial.println("[ENC] OK");

    // ADC
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Driver
    driver.pwm_frequency        = 20000;
    driver.voltage_power_supply = 15.0f;
    driver.voltage_limit        = 15.0f;
    driver.dead_zone            = 0.0f;
    if (!driver.init()) {
        Serial.println("[DRV] Driver init FAILED — halting.");
        while(1);
    }
    digitalWrite(DRV_EN, HIGH);

    // Current sense
    // Link driver so SimpleFOC knows PWM timing for synchronised sampling
    currentSense.linkDriver(&driver);

    // Set ADC attenuation for 0-3100mV range on all three sense pins
    // ADC_11db gives 0-2450mV on ESP32-S3 by default but with calibration
    // reaches ~3100mV — SimpleFOC handles this via analogRead internally
    analogReadResolution(12);
    analogRead(SOA_PIN);  // Initialises pin before setting attenuation
    analogRead(SOB_PIN);
    analogRead(SOC_PIN);
    analogSetPinAttenuation(SOA_PIN, ADC_11db);
    analogSetPinAttenuation(SOB_PIN, ADC_11db);
    analogSetPinAttenuation(SOC_PIN, ADC_11db);

    // Init current sense — auto-calibrates Vref with motor stationary
    // Samples ADC at zero current to find midpoint automatically
    if (!currentSense.init()) {
        Serial.println("[CS] Current sense init FAILED — halting.");
        while(1);
    }
    Serial.println("[CS] Current sense calibrated OK.");

    // Print Vref found by calibration
    Serial.printf("[CS] Vref phase A: %.4f\n", currentSense.offset_ia);
    Serial.printf("[CS] Vref phase B: %.4f\n", currentSense.offset_ib);
    Serial.printf("[CS] Vref phase C: %.4f\n", currentSense.offset_ic);

    // Motor
    motor.linkSensor(&sensor);
    motor.linkDriver(&driver);
    motor.linkCurrentSense(&currentSense);

    // FOC current torque control
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller        = MotionControlType::torque;

    // Current PID — Q axis (torque producing)
    // Bandwidth should be ~10x lower than PWM frequency
    // At 25kHz PWM, target ~2.5kHz current loop bandwidth
    motor.PID_current_q.P           = 0.15f;
    motor.PID_current_q.I           = 50.0f;
    motor.PID_current_q.D           = 0.0f;
    motor.PID_current_q.limit       = 15.0f;
    motor.PID_current_q.output_ramp = 1000.0f;
    motor.PID_current_d.P           = 0.15f;
    motor.PID_current_d.I           = 50.0f;
    motor.PID_current_d.D           = 0.0f;
    motor.PID_current_d.limit       = 15.0f;
    motor.PID_current_d.output_ramp = 1000.0f;

    motor.LPF_current_q.Tf          = 0.005f;
    motor.LPF_current_d.Tf          = 0.005f;

    motor.voltage_sensor_align = 1.0f;
    motor.voltage_limit        = 12.0f;
    motor.current_limit        = I_LIMIT;
    motor.velocity_limit       = 20.0f;

    motor.useMonitoring(Serial);
    motor.init();

    // initFOC — also aligns current sense phase directions automatically
    Serial.println("[FOC] Running initFOC...");
    if (!motor.initFOC()) {
        Serial.println("[FOC] initFOC FAILED — halting.");
        while(1);
    }
    Serial.printf("[FOC] Zero angle: %.4f  Direction: %s\n",
                  motor.zero_electric_angle,
                  motor.sensor_direction == Direction::CW ? "CW" : "CCW");
    Serial.println("[FOC] OK.");

    xTaskCreatePinnedToCore(
        motorRuntime,
        "MotorTask",
        10000,
        NULL,
        5,                  // Priority 5
        &MotorTaskHandle,
        0);                 // Core 0

    // Commander
    // T command sets target current in Amps
    // e.g. T2.0 -> 2A torque
    //      T0   -> stop
    //      T-2  -> reverse torque
    command.add('T', onTarget, "target voltage (V)");
    command.add('M', [](char* cmd){ command.motor(&motor, cmd); }, "motor");

    Serial.println("=== READY ===");
    Serial.println("Commands:");
    Serial.printf("  T%.1f  -> max torque (7A)\n", I_LIMIT);
    Serial.println("  T2.0  -> 2A torque");
    Serial.println("  T0    -> stop");
    Serial.println("  T-2   -> reverse torque");

    command.add('P', [](char* cmd){ command.scalar(&motor.PID_current_q.P, cmd); }, "P gain");
    command.add('I', [](char* cmd){ command.scalar(&motor.PID_current_q.I, cmd); }, "I gain");
    command.add('F', [](char* cmd){ command.scalar(&motor.LPF_current_q.Tf, cmd); }, "LPF Tf");

    // Sync D-axis to Q-axis (Optional but recommended for current sensing)
    // This makes sure both axes stay identical while you tune.
    command.add('S', [](char* cmd){ 
    motor.PID_current_d.P = motor.PID_current_q.P;
    motor.PID_current_d.I = motor.PID_current_q.I;
    motor.LPF_current_d.Tf = motor.LPF_current_q.Tf;
    Serial.println("D-axis synced to Q-axis");
}, "sync D-axis");
}

// ================================================================
//  LOOP
// ================================================================
// ================================================================
//  LOOP
// ================================================================

void loop() {
    command.run();

    static uint32_t last = 0;
    if (millis() - last > 500) {
        // 1. Disable PWM temporarily to get a "pure" noise reading
        // If the noise persists with the driver OFF, it's EMI or Ground noise.
        driver.disable(); 
        delayMicroseconds(50); // Small pause for MOSFETs to close

        // 2. Get Raw Phase Readings (in Amps)
        PhaseCurrent_s phases = currentSense.getPhaseCurrents();

        // 3. Get Raw ADC Millivolts (Directly from ESP32 pins)
        // This lets us see if the 0-Amp reference (Vref) is drifting.
        uint32_t mvA = analogReadMilliVolts(SOA_PIN);
        uint32_t mvB = analogReadMilliVolts(SOB_PIN);
        uint32_t mvC = analogReadMilliVolts(SOC_PIN);

        // 4. Print everything for the "Hand Spin" test
        Serial.printf("DRV OFF | PhA: %.3fA | PhB: %.3fA | ADC_A: %dmV | ADC_B: %dmV | Vel: %.1f\n",
                      phases.a, phases.b, mvA, mvB, motor.shaft_velocity);

        // 5. Re-enable the driver so it can continue functioning
        driver.enable();

        last = millis();
    }
}

// void loop() {
//     command.run();

//     static uint32_t last = 0;
//     if (millis() - last > 100) {
        
//         // 1. Update Battery Voltage (Required for display_V)
//         uint32_t totalMilliVolts = 0;
//         for (int i = 0; i < SAMPLES; i++) {
//             totalMilliVolts += analogReadMilliVolts(VSENSE_PIN);
//         }
//         float avgMilliVolts = (float)totalMilliVolts / SAMPLES;
//         display_V = (avgMilliVolts / 1000.0) * dividerRatio * calibrationTrim;

//         // 2. Get DQ currents
//         DQCurrent_s foc = currentSense.getFOCCurrents(motor.electrical_angle);
//         float totalCurrent = hypot(foc.q, foc.d);

//         // 3. Serial Logging
//         Serial.printf("Batt: %.2fV | Target: %.2fV | Measured: %.2fA | Iq: %.2fA | Id: %.2fA | Vel: %.1f\n",
//                       display_V, target, totalCurrent, foc.q, foc.d, motor.shaft_velocity);

//         // 4. THE FIX: Only talk to the DRV8323 if the hardware fault pin triggers
//         if (digitalRead(nFAULT) == LOW) {
//             // Now, Core 1 only steals the SPI bus if there is a literal emergency
//             uint16_t f1 = drvReadSpi(0x00);
//             uint16_t f2 = drvReadSpi(0x01);
//             Serial.printf(">>> [DRV FAULT] 0x%03X : 0x%03X <<<\n", f1, f2);
//         }

//         // 5. Update the OLED display 
//         u8g2.clearBuffer();                 
//         u8g2.drawStr(0, 15, "GL40 STATUS");
//         u8g2.drawHLine(0, 18, 128);
        
//         u8g2.setCursor(0, 40);
//         u8g2.print("Batt:  "); u8g2.print(display_V, 1); u8g2.print(" V");
        
//         u8g2.setCursor(0, 60);
//         u8g2.print("Load:  "); u8g2.print(totalCurrent, 2); u8g2.print(" A");
//         u8g2.sendBuffer();

//         last = millis();
//     }
// }