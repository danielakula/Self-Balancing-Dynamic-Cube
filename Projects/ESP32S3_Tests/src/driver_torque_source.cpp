#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <esp_task_wdt.h>
#include <soc/gpio_struct.h>

TaskHandle_t MotorTaskHandle;
SemaphoreHandle_t spiMutex = NULL;

volatile bool pauseFOCForSPI = false;

#define TEST_POINT_PIN 44

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

#define VSENSE_PIN 8      
#define SAMPLES 64        
const float dividerRatio = 11.0; 
const float calibrationTrim = 1.008338;

#define MASTER_SDA 4
#define MASTER_SCL 5

// Initialize the display 
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE, MASTER_SCL, MASTER_SDA);

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
        digitalWrite(ENC1_CS, HIGH);
        uint16_t frame = ((uint16_t)(address & 0x0F) << 11) | (data & 0x7FF);
        hwSpi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
        digitalWrite(DRV_CS, LOW);
        delayMicroseconds(1);
        hwSpi.transfer16(frame);
        delayMicroseconds(1);
        digitalWrite(DRV_CS, HIGH);
        hwSpi.endTransaction();
        xSemaphoreGive(spiMutex); 
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
}

// ================================================================
//  SIMPLEFOC OBJECTS
// ================================================================
BLDCDriver6PWM driver = BLDCDriver6PWM(AH_PIN, AL_PIN, BH_PIN, BL_PIN, CH_PIN, CL_PIN);
BLDCMotor      motor  = BLDCMotor(14, 0.5f, 195.0f, 0.00018f);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, ENC1_CS);
LowsideCurrentSense currentSense = LowsideCurrentSense(R_SHUNT, CSA_GAIN, SOA_PIN, SOB_PIN, SOC_PIN);

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

// 1. Instantiate the Commander object
Commander command = Commander(Serial);

// 2. Define the callback functions for the Commander
// These link the incoming Serial data to your specific motor variables
void onPid_Q(char* cmd) { command.pid(&motor.PID_current_q, cmd); }
void onLpf_Q(char* cmd) { command.lpf(&motor.LPF_current_q, cmd); }

void onTarget(char* cmd) { command.scalar(&target, cmd); }

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
    command.add('C', onPid_Q, "Current Q-axis PID");
    command.add('L', onLpf_Q, "Current Q-axis LPF");
    
    Serial.println("Tuning Ready! Type C or L in the monitor.");

    spiMutex = xSemaphoreCreateMutex();

    pinMode(DRV_CS,  OUTPUT); digitalWrite(DRV_CS,  HIGH);
    pinMode(ENC1_CS, OUTPUT); digitalWrite(ENC1_CS, HIGH);
    pinMode(DRV_EN,  OUTPUT); digitalWrite(DRV_EN,  HIGH);
    pinMode(nFAULT,  INPUT_PULLUP);

    hwSpi.begin(MASTER_SCK, MASTER_MISO, MASTER_MOSI, -1);
    u8g2.begin();
    u8g2.setFont(u8g2_font_7x14_tr);

    initDRV8323S();
    // Maximize SPI speed to 20MHz to slash sensor read times
    sensor.clock_speed = 20000000; 
    sensor.init(&hwSpi);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    driver.pwm_frequency        = 12500;
    driver.voltage_power_supply = 15.0f;
    driver.voltage_limit        = 14.0f;
    driver.dead_zone            = 0.08f;
    driver.init();
    digitalWrite(DRV_EN, HIGH);

    currentSense.linkDriver(&driver);
    analogRead(SOA_PIN);  
    analogRead(SOB_PIN);
    analogRead(SOC_PIN);
    analogSetPinAttenuation(SOA_PIN, ADC_11db);
    analogSetPinAttenuation(SOB_PIN, ADC_11db);
    analogSetPinAttenuation(SOC_PIN, ADC_11db);
    currentSense.init();

    motor.linkSensor(&sensor);
    motor.linkDriver(&driver);
    motor.linkCurrentSense(&currentSense);

    motor.torque_controller = TorqueControlType::foc_current;
    motor.controller        = MotionControlType::torque;
    motor.feed_forward_velocity = true;
    
    float R = 0.5;           
    float L = 0.00018;
    float bandwidth_hz =400.0; // loopFOC() ~10kHz, L*2pi*bandwidth=R, reduced to ensure stability
    float omega = _2PI * bandwidth_hz;
    float calc_P = L * omega;
    float calc_I = R * omega;
    float calc_Tf = 1.0 / (omega * 5.0);

    motor.PID_current_q.P = calc_P;
    motor.PID_current_q.I = calc_I;
    motor.PID_current_d.P = calc_P;
    motor.PID_current_d.I = calc_I;
    motor.LPF_current_q.Tf = calc_Tf; 
    motor.LPF_current_d.Tf = calc_Tf;

    // motor.PID_current_q.P           = 0.17f;
    // motor.PID_current_q.I           = 470.0f;
    // motor.PID_current_d.P           = 0.17f;
    // motor.PID_current_d.I           = 470.0f;

    //motor.LPF_current_q.Tf          = 0.00021f;
    //motor.LPF_current_d.Tf          = 0.00021f;

    motor.PID_current_d.output_ramp = 10000.0f;
    motor.PID_current_q.output_ramp = 10000.0f;
    motor.PID_current_d.D           = 0.0f;
    motor.PID_current_q.D           = 0.0f;
    motor.PID_current_d.limit       = 15.0f;
    motor.PID_current_q.limit       = 15.0f;

    motor.voltage_sensor_align = 1.0f;
    motor.voltage_limit  = 14.0f;
    motor.current_limit  = I_LIMIT;
    motor.velocity_limit = 10.0f;

    motor.useMonitoring(Serial);
    motor.init();
    motor.initFOC();
    
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

    static uint32_t last = 0;
    if (millis() - last > 25) {
        
        uint32_t totalMilliVolts = 0;
        for (int i = 0; i < SAMPLES; i++) {
            totalMilliVolts += analogReadMilliVolts(VSENSE_PIN);
        }
        float avgMilliVolts = (float)totalMilliVolts / SAMPLES;
        display_V = (avgMilliVolts / 1000.0) * dividerRatio * calibrationTrim;

        float current_q = motor.current.q; // Already calculated by Core 0!
        float current_d = motor.current.d;
        float totalCurrent = hypot(current_q, current_d);

        Serial.printf("%.2f, %.2f, %.2f\n",current_q, current_d, target); 
        // Serial.printf("Batt: %.2fV | Iq: %.2fA | Target: %.2fA | Vel: %.1f\n",
        //       display_V, current_q, target, motor.shaft_velocity);

        if (digitalRead(nFAULT) == LOW) {
            pauseFOCForSPI = true; // Tell Core 0 to pause
            delay(1);              // Give Core 0 a moment to finish its current loop
            
            uint16_t f1 = drvReadSpi(0x00);
            uint16_t f2 = drvReadSpi(0x01);
            Serial.printf(">>> [DRV FAULT] 0x%03X : 0x%03X <<<\n", f1, f2);
            
            pauseFOCForSPI = false; // Release the bus
        }

        u8g2.clearBuffer();                 
        u8g2.drawStr(0, 15, "GL40 STATUS");
        u8g2.drawHLine(0, 18, 128);
        u8g2.setCursor(0, 40);
        u8g2.print("Batt:  "); u8g2.print(display_V, 1); u8g2.print(" V");
        u8g2.setCursor(0, 60);
        u8g2.print("Load:  "); u8g2.print(totalCurrent, 2); u8g2.print(" A");
        u8g2.sendBuffer();

        last = millis();
    }

    // Feed the Core 0 Watchdog! 
    // Because command.run() loops instantly, Core 0 will panic without this.
    vTaskDelay(10 / portTICK_PERIOD_MS);
}