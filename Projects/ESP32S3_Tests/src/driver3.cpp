#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>
#include <esp_task_wdt.h>

TaskHandle_t MotorTaskHandle;
SemaphoreHandle_t spiMutex = NULL;

volatile bool pauseFOCForSPI = false;

#define SLAVE_SCK   4
#define SLAVE_MISO  6
#define SLAVE_MOSI  5

// ================================================================
//  DRIVER 3 DEFINITIONS
// ================================================================
#define ENC3_CS     41
#define DRV3_CS     10
#define DRV3_EN     20
#define nFAULT3     19

#define AH3_PIN     8
#define AL3_PIN     18
#define BH3_PIN     17
#define BL3_PIN     16
#define CH3_PIN     15
#define CL3_PIN     7

#define SOA3_PIN    11
#define SOB3_PIN    9
#define SOC3_PIN    3

#define VSENSE_PIN 8      
#define SAMPLES 64        
const float dividerRatio = 11.0; 
const float calibrationTrim = 1.008338;

#define MASTER_SDA 4
#define MASTER_SCL 5

float display_V = 0;
float display_I = 0;

// ================================================================
//  CURRENT SENSE PARAMETERS
// ================================================================
#define R_SHUNT     0.010f   // 10 mOhm
#define CSA_GAIN    20.0f    // 20 V/V
#define I_LIMIT     1.0f     // Amps

// ================================================================
//  SPI — single bus, manual CS
// ================================================================
SPIClass hwSpi(1);

// ================================================================
//  DRV8323S
// ================================================================
void drvWriteSpi(uint8_t address, uint16_t data) {
    if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        digitalWrite(ENC3_CS, HIGH);
        uint16_t frame = ((uint16_t)(address & 0x0F) << 11) | (data & 0x7FF);
        hwSpi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
        digitalWrite(DRV3_CS, LOW);
        delayMicroseconds(1);
        hwSpi.transfer16(frame);
        delayMicroseconds(1);
        digitalWrite(DRV3_CS, HIGH);
        hwSpi.endTransaction();
        xSemaphoreGive(spiMutex); 
    }
}

uint16_t drvReadSpi(uint8_t address) {
    uint16_t result = 0;
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
        digitalWrite(ENC3_CS, HIGH);
        uint16_t frame = (1 << 15) | ((uint16_t)(address & 0x0F) << 11);
        hwSpi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
        digitalWrite(DRV3_CS, LOW);
        delayMicroseconds(1);
        result = hwSpi.transfer16(frame);
        delayMicroseconds(1);
        digitalWrite(DRV3_CS, HIGH);
        hwSpi.endTransaction();
        xSemaphoreGive(spiMutex);
    }
    return result & 0x7FF;
}

void initDRV8323S() {
    Serial.println("[DRV] Configuring...");
    digitalWrite(DRV3_EN, LOW);  delay(50);
    digitalWrite(DRV3_EN, HIGH); delay(100);

    drvWriteSpi(0x02, 0b00000000000);
    drvWriteSpi(0x03, 0b01100100010);
    drvWriteSpi(0x04, 0b10100100010);
    drvWriteSpi(0x05, 0b01000011001);
    drvWriteSpi(0x06, 0b01110000011);
}

// ================================================================
//  SIMPLEFOC OBJECTS
// ================================================================
BLDCDriver6PWM driver = BLDCDriver6PWM(AH3_PIN, AL3_PIN, BH3_PIN, BL3_PIN, CH3_PIN, CL3_PIN);
BLDCMotor      motor  = BLDCMotor(14, 0.5f, 195.0f, 0.00018f);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, ENC3_CS);
LowsideCurrentSense currentSense = LowsideCurrentSense(R_SHUNT, CSA_GAIN, SOA3_PIN, SOB3_PIN, SOC3_PIN);

Commander command = Commander(Serial);
float target = 0.0f;  

bool testRunning = false;
float testTargetCurrent = 0.2f; // 1 Amp constant pull
float maxSafeSpeed = 150.0f;    // Rad/s (Adjust based on your wheel's physical limits!)

void onRunTest(char* cmd) {
    if (!testRunning) {
        target = testTargetCurrent; 
        testRunning = true;
        Serial.println("Target_I,Measured_I_q,Measured_I_d,Speed_rads"); // CSV Header
    }
}

bool kvTestRunning = false;
uint32_t testStartTime = 0;
float testVoltage = 3.0f; // 3.0V is safe and will yield ~600 RPM on a 200KV motor

void onRunKVTest(char* cmd) {
    if (!kvTestRunning) {
        // Temporarily override the controller to pure voltage mode for the test
        motor.torque_controller = TorqueControlType::voltage;
        target = testVoltage; 
        kvTestRunning = true;
        testStartTime = millis();
        Serial.println("=== STARTING KV TEST ===");
        Serial.printf("Applying %.1f Volts. Please wait 5 seconds...\n", testVoltage);
    }
}

void onTarget(char* cmd) { command.scalar(&target, cmd); }

// ================================================================
//  THE UNTHROTTLED FOC TASK (CORE 1)
// ================================================================
void motorRuntime(void * pvParameters) {
    disableCore0WDT(); 
    TaskHandle_t idleTaskHandle = xTaskGetIdleTaskHandleForCPU(0);
    if (idleTaskHandle != NULL) {
        esp_task_wdt_delete(idleTaskHandle);
    }
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

    for(;;) {
        // Use the ultra-fast boolean flag instead of the heavy FreeRTOS Mutex
        if (!pauseFOCForSPI) {
            motor.loopFOC();
            motor.move(target);
        }
        // NO DELAY. NO MUTEX. PURE SPEED.
    }
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(3000);
    Serial.println("=== GL40 TORQUE CONTROL — 10kHz GOVERNOR ===");

    command.add('T', onTarget, "target current (A)");
    command.add('R', onRunTest, "Run Ramp Test"); 
    command.add('K', onRunKVTest, "Run True KV Test"); // Add this!

    spiMutex = xSemaphoreCreateMutex();

    pinMode(DRV3_CS,  OUTPUT); digitalWrite(DRV3_CS,  HIGH);
    pinMode(ENC3_CS, OUTPUT); digitalWrite(ENC3_CS, HIGH);
    pinMode(DRV3_EN,  OUTPUT); digitalWrite(DRV3_EN,  HIGH);
    pinMode(nFAULT3,  INPUT_PULLUP);

    hwSpi.begin(SLAVE_SCK, SLAVE_MISO, SLAVE_MOSI, -1);

    initDRV8323S();
    // Maximize SPI speed to 20MHz to slash sensor read times
    sensor.clock_speed = 20000000; 
    sensor.init(&hwSpi);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    driver.pwm_frequency        = 15000;
    driver.voltage_power_supply = 15.0f;
    driver.voltage_limit        = 14.0f;
    driver.dead_zone            = 0.001f;
    driver.init();
    digitalWrite(DRV3_EN, HIGH);

    currentSense.linkDriver(&driver);
    analogRead(SOA3_PIN);  
    analogRead(SOB3_PIN);
    analogRead(SOC3_PIN);
    analogSetPinAttenuation(SOA3_PIN, ADC_11db);
    analogSetPinAttenuation(SOB3_PIN, ADC_11db);
    analogSetPinAttenuation(SOC3_PIN, ADC_11db);
    currentSense.init();

    motor.linkSensor(&sensor);
    motor.linkDriver(&driver);
    motor.linkCurrentSense(&currentSense);

    motor.torque_controller = TorqueControlType::foc_current;
    motor.controller        = MotionControlType::torque;
    motor.feed_forward_velocity = true;

    motor.PID_current_q.P           = 0.3f;
    motor.PID_current_q.I           = 50.0f;
    motor.PID_current_q.D           = 0.0f;
    motor.PID_current_q.limit       = 15.0f;
    motor.PID_current_q.output_ramp = 1000.0f;
    motor.PID_current_d.P           = 0.3f;
    motor.PID_current_d.I           = 50.0f;
    motor.PID_current_d.D           = 0.0f;
    motor.PID_current_d.limit       = 15.0f;
    motor.PID_current_d.output_ramp = 1000.0f;

    motor.LPF_current_q.Tf          = 0.01f;
    motor.LPF_current_d.Tf          = 0.01f;

    motor.voltage_sensor_align = 1.0f;
    motor.voltage_limit  = 14.0f;
    motor.current_limit  = I_LIMIT;
    motor.velocity_limit = 10.0f;

    motor.useMonitoring(Serial);
    motor.init();
    motor.initFOC();

    command.add('T', onTarget, "target current (A)");
    
    xTaskCreatePinnedToCore(
        motorRuntime,
        "MotorTask",
        10000,
        NULL,
        5,                  
        &MotorTaskHandle,
        0);          
}

// ================================================================
//  LOOP (CORE 0)
// ================================================================
void loop() {
    command.run();
    bool ledState = LOW;

    static uint32_t last = 0;
    if (millis() - last > 500) {

        // 2. Get DQ currents
        DQCurrent_s foc = currentSense.getFOCCurrents(motor.electrical_angle);
        float totalCurrent = hypot(foc.q, foc.d);

        // 3. Serial Logging
        Serial.printf("Iq: %.2fA | Id: %.2fA | Target: %.2fA | Vel: %.1f\n",
             foc.q, foc.d, target, motor.shaft_velocity);

        // 4. THE FIX: Only talk to the DRV8323 if the hardware fault pin triggers
        if (digitalRead(nFAULT3) == LOW) {
            // Now, Core 1 only steals the SPI bus if there is a literal emergency
            uint16_t f1 = drvReadSpi(0x00);
            uint16_t f2 = drvReadSpi(0x01);
            Serial.printf(">>> [DRV FAULT] 0x%03X : 0x%03X <<<\n", f1, f2);
        }

        last = millis();
    }
}