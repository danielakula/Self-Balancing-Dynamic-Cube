#include <Arduino.h>
#include <SimpleFOC.h>

// ================================================================
//  PIN DEFINITIONS
// ================================================================
#define MASTER_SCK  37
#define MASTER_MISO 39
#define MASTER_MOSI 38
#define ENC1_CS     36
#define DRV_CS      48
#define DRV_EN      35
#define nFAULT      45

#define AH_PIN  47
#define AL_PIN  21
#define BH_PIN  14
#define BL_PIN  13
#define CH_PIN  12
#define CL_PIN  11

#define SOA_PIN  3
#define SOB_PIN  9
#define SOC_PIN  10

#define R_SHUNT  0.010f
#define CSA_GAIN 20.0f
#define I_LIMIT  5.0f

// ================================================================
//  SINGLE SPI BUS — shared by DRV and encoder via manual CS
// ================================================================
SPIClass hwSpi(1);

// ================================================================
//  DRV8323S
// ================================================================
void drvWrite(uint8_t address, uint16_t data) {
    digitalWrite(ENC1_CS, HIGH);  // Deselect encoder
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

uint16_t drvRead(uint8_t address) {
    digitalWrite(ENC1_CS, HIGH);  // Deselect encoder
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

void initDRV() {
    digitalWrite(DRV_EN, LOW);  delay(50);
    digitalWrite(DRV_EN, HIGH); delay(100);
    drvWrite(0x02, 0b00000000000);
    drvWrite(0x03, 0b01100100010);
    drvWrite(0x04, 0b10100100010);
    drvWrite(0x05, 0b01000011001);
    drvWrite(0x06, 0b01110000001);
    Serial.printf("[DRV] Reg02:0x%03X 03:0x%03X 04:0x%03X 05:0x%03X 06:0x%03X\n",
                  drvRead(0x02), drvRead(0x03), drvRead(0x04),
                  drvRead(0x05), drvRead(0x06));
    Serial.printf("[DRV] Fault1:0x%03X Fault2:0x%03X\n",
                  drvRead(0x00), drvRead(0x01));
}

// ================================================================
//  SIMPLEFOC OBJECTS
// ================================================================
BLDCDriver6PWM      driver       = BLDCDriver6PWM(AH_PIN, AL_PIN, BH_PIN, BL_PIN, CH_PIN, CL_PIN);
BLDCMotor           motor        = BLDCMotor(14, 0.5f, 210.0f, 0.00018f);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, ENC1_CS);
LowsideCurrentSense currentSense = LowsideCurrentSense(R_SHUNT, CSA_GAIN, SOA_PIN, SOB_PIN, SOC_PIN);
Commander           command      = Commander(Serial);

float target = 0.0f;
void onTarget(char* cmd) { command.scalar(&target, cmd); }

// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== GL40 ANGLE CONTROL — FOC CURRENT + MCPWM ===");

    pinMode(DRV_CS,  OUTPUT); digitalWrite(DRV_CS,  HIGH);
    pinMode(ENC1_CS, OUTPUT); digitalWrite(ENC1_CS, HIGH);
    pinMode(DRV_EN,  OUTPUT); digitalWrite(DRV_EN,  HIGH);
    pinMode(nFAULT,  INPUT_PULLUP);

    hwSpi.begin(MASTER_SCK, MASTER_MISO, MASTER_MOSI, -1);

    initDRV();

    sensor.init(&hwSpi);
    Serial.println("[ENC] OK");

    // ADC
    analogReadResolution(12);
    analogRead(SOA_PIN);
    analogRead(SOB_PIN);
    analogRead(SOC_PIN);
    analogSetPinAttenuation(SOA_PIN, ADC_11db);
    analogSetPinAttenuation(SOB_PIN, ADC_11db);
    analogSetPinAttenuation(SOC_PIN, ADC_11db);

    // Driver
    driver.pwm_frequency        = 20000;
    driver.voltage_power_supply = 15.3f;
    driver.voltage_limit        = 15.3f;
    driver.dead_zone            = 0.05f;
    if (!driver.init()) {
        Serial.println("[DRV] Driver init FAILED"); while(1);
    }
    digitalWrite(DRV_EN, HIGH);

    // Current sense
    currentSense.linkDriver(&driver);
    if (!currentSense.init()) {
        Serial.println("[CS] Current sense FAILED"); while(1);
    }
    Serial.printf("[CS] Vref A:%.4f B:%.4f C:%.4f\n",
                  currentSense.offset_ia,
                  currentSense.offset_ib,
                  currentSense.offset_ic);

    // Motor
    motor.linkSensor(&sensor);
    motor.linkDriver(&driver);
    motor.linkCurrentSense(&currentSense);

    motor.torque_controller = TorqueControlType::foc_current;
    motor.controller        = MotionControlType::angle;

    // Angle PID 
    motor.PID_velocity.P           = 0.1f;
    motor.PID_velocity.I           = 1.0f;
    motor.PID_velocity.D           = 0.0f;
    motor.PID_velocity.output_ramp = 100.0f;
    motor.PID_velocity.limit       = I_LIMIT;
    motor.LPF_velocity.Tf          = 0.01f;
    motor.P_angle.P = 10.0f;

    // Current PID 
    motor.PID_current_q.P           = 0.5f;
    motor.PID_current_q.I           = 50.0f;
    motor.PID_current_q.D           = 0.0f;
    motor.PID_current_q.limit       = 15.0f;
    motor.PID_current_q.output_ramp = 1000.0f;
    motor.PID_current_d.P           = 0.5f;
    motor.PID_current_d.I           = 50.0f;
    motor.PID_current_d.D           = 0.0f;
    motor.PID_current_d.limit       = 15.0f;
    motor.PID_current_d.output_ramp = 1000.0f;

    motor.LPF_current_q.Tf          = 0.0005f;
    motor.LPF_current_d.Tf          = 0.0005f;

    motor.voltage_sensor_align = 1.0f;
    motor.voltage_limit        = 15.0f;
    motor.current_limit        = I_LIMIT;
    motor.velocity_limit       = 20.0f;

    motor.useMonitoring(Serial);
    motor.init();

    Serial.println("[FOC] initFOC...");
    if (!motor.initFOC()) {
        Serial.println("[FOC] FAILED — halting");
        while(1);
    }
    Serial.printf("[FOC] Zero angle:%.4f  Direction:%s\n",
                  motor.zero_electric_angle,
                  motor.sensor_direction == Direction::CW ? "CW" : "CCW");
    Serial.println("[FOC] OK");

    target = motor.shaft_angle;

    command.add('T', onTarget, "target angle (rad)");
    command.add('M', [](char* cmd){ command.motor(&motor, cmd); }, "motor");

    Serial.println("=== READY ===");
    Serial.println("  T0     -> hold at 0 rad");
    Serial.println("  T1.57  -> move to 90 deg");
    Serial.println("  T3.14  -> move to 180 deg");
    Serial.println("  T6.28  -> move to 360 deg");
    Serial.println("  MVC3.0  -> angle P gain");
    Serial.println("  MVCD0.3 -> angle D gain");
    Serial.println("  MMCP2.0 -> current P gain");
    Serial.println("  MMCI100 -> current I gain");
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
    motor.loopFOC();
    motor.move(target);
    command.run();
}


    // static uint32_t lastPrint = 0;
    // if (millis() - lastPrint > 500) {
    //     DQCurrent_s foc = currentSense.getFOCCurrents(motor.electrical_angle);
    //     Serial.printf("[LOOP] target=%.4f  actual=%.4f  err=%.3f deg  Iq=%.3fA  Id=%.3fA  vel=%.3f rad/s\n",
    //                   target,
    //                   motor.shaft_angle,
    //                   (target - motor.shaft_angle) * RAD_TO_DEG,
    //                   foc.q,
    //                   foc.d,
    //                   motor.shaft_velocity);

    //     if (!digitalRead(nFAULT)) {
    //         Serial.printf("[DRV] Fault1:0x%03X Fault2:0x%03X\n",
    //                       drvRead(0x00), drvRead(0x01));
    //         digitalWrite(DRV_EN, LOW); delay(10);
    //         digitalWrite(DRV_EN, HIGH); delay(10);
    //         initDRV();
    //         Serial.println("[DRV] Cleared.");
    //     }
    //     lastPrint = millis();
    // }