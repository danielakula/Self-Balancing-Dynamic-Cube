#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

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

// ================================================================
//  SPI — single bus, manual CS
// ================================================================
SPIClass hwSpi(1);

// ================================================================
//  DRV8323S — configure registers only, no ISR
// ================================================================
void drvWriteSpi(uint8_t address, uint16_t data) {
    digitalWrite(ENC1_CS, HIGH);
    uint16_t frame = ((uint16_t)(address & 0x0F) << 11) | (data & 0x7FF);
    hwSpi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    digitalWrite(DRV_CS, LOW);
    delayMicroseconds(1);
    hwSpi.transfer16(frame);
    delayMicroseconds(1);
    digitalWrite(DRV_CS, HIGH);
    hwSpi.endTransaction();
    delayMicroseconds(2);
}

uint16_t drvReadSpi(uint8_t address) {
    digitalWrite(ENC1_CS, HIGH);
    uint16_t frame = (1 << 15) | ((uint16_t)(address & 0x0F) << 11);
    hwSpi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    digitalWrite(DRV_CS, LOW);
    delayMicroseconds(1);
    uint16_t result = hwSpi.transfer16(frame);
    delayMicroseconds(1);
    digitalWrite(DRV_CS, HIGH);
    hwSpi.endTransaction();
    delayMicroseconds(2);
    return result & 0x7FF;
}

void initDRV8323S() {
    Serial.println("[DRV] Configuring...");

    // Power cycle EN
    digitalWrite(DRV_EN, LOW);
    delay(50);
    digitalWrite(DRV_EN, HIGH);
    delay(100);

    // Write registers
    drvWriteSpi(0x02, 0b00000000000);
    drvWriteSpi(0x03, 0b01100100010);
    drvWriteSpi(0x04, 0b10100100010);
    drvWriteSpi(0x05, 0b01010010001);
    drvWriteSpi(0x06, 0b01101000011);

    // Verify
    Serial.printf("[DRV] Reg02: 0x%03X\n", drvReadSpi(0x02));
    Serial.printf("[DRV] Reg03: 0x%03X\n", drvReadSpi(0x03));
    Serial.printf("[DRV] Reg04: 0x%03X\n", drvReadSpi(0x04));
    Serial.printf("[DRV] Reg05: 0x%03X\n", drvReadSpi(0x05));
    Serial.printf("[DRV] Reg06: 0x%03X\n", drvReadSpi(0x06));
    Serial.printf("[DRV] Fault1: 0x%03X  Fault2: 0x%03X\n",
                  drvReadSpi(0x00), drvReadSpi(0x01));
}

// ================================================================
//  SIMPLEFOC — straight from example
// ================================================================
BLDCDriver6PWM driver = BLDCDriver6PWM(AH_PIN, AL_PIN, BH_PIN, BL_PIN, CH_PIN, CL_PIN);
BLDCMotor      motor  = BLDCMotor(14, 0.5f, 210.0f, 0.00018f);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, ENC1_CS);
Commander      command = Commander(Serial);

float target = 0.0f;
void onTarget(char* cmd) { command.scalar(&target, cmd); }

void setup() {
    Serial.begin(115200);
    delay(2000);

    // Pins — NO nFAULT interrupt
    pinMode(DRV_CS,  OUTPUT); digitalWrite(DRV_CS,  HIGH);
    pinMode(ENC1_CS, OUTPUT); digitalWrite(ENC1_CS, HIGH);
    pinMode(DRV_EN,  OUTPUT); digitalWrite(DRV_EN,  HIGH); // EN high from the start
    pinMode(nFAULT,  INPUT_PULLUP);

    // SPI
    hwSpi.begin(MASTER_SCK, MASTER_MISO, MASTER_MOSI, -1);

    // DRV — configure before anything else touches the bus
    initDRV8323S();

    // Encoder
    sensor.init(&hwSpi);
    Serial.println("[ENC] OK");

    Serial.println("[ENC] Rotate motor by hand:");
    for (int i = 0; i < 15; i++) {
        sensor.update();
        Serial.printf("  angle=%.4f  raw=%d\n", sensor.getAngle(), sensor.readRawAngle());
        delay(200);
    }

    // Driver — straight from SimpleFOC example
    driver.pwm_frequency        = 25000;
    driver.voltage_power_supply = 15.0f;
    driver.voltage_limit        = 15.0f;
    driver.dead_zone            = 0.05f;
    driver.init();

    // Motor — straight from SimpleFOC example
    motor.linkSensor(&sensor);
    motor.linkDriver(&driver);
    motor.torque_controller        = TorqueControlType::voltage;
    motor.controller               = MotionControlType::velocity;
    motor.PID_velocity.P           = 0.2f;
    motor.PID_velocity.I           = 2.0f;
    motor.PID_velocity.D           = 0.0f;
    motor.PID_velocity.output_ramp = 300.0f;
    motor.PID_velocity.limit       = 10.0f;
    motor.LPF_velocity.Tf          = 0.01f;
    motor.voltage_sensor_align     = 2.0f;
    motor.voltage_limit            = 5.0f;
    motor.velocity_limit           = 200.0f;
    motor.useMonitoring(Serial);
    motor.init();

    // initFOC
    Serial.println("[FOC] Running initFOC...");
    motor.initFOC();
    Serial.printf("[FOC] Zero angle: %.4f  Direction: %s\n",
                  motor.zero_electric_angle,
                  motor.sensor_direction == Direction::CW ? "CW" : "CCW");

    // Raise limits after alignment
    motor.voltage_limit      = 5.0f;
    motor.PID_velocity.limit = 5.0f;

    command.add('T', onTarget, "target velocity (rad/s)");
    command.add('M', [](char* cmd){ command.motor(&motor, cmd); }, "motor");

    Serial.println("=== READY — Send T<value> for velocity in rad/s ===");
}

void loop() {
    motor.loopFOC();
    motor.move(target);
    command.run();

    static uint32_t last = 0;
    if (millis() - last > 500) {
        Serial.printf("[LOOP] target=%.2f  vel=%.2f  Uq=%.3f  nFAULT=%d\n",
                      target, motor.shaft_velocity, motor.voltage.q,
                      digitalRead(nFAULT));  // Poll instead of ISR
        last = millis();
    }
}