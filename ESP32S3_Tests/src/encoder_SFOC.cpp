#include <Arduino.h>
#include <SimpleFOC.h>

// Pin Definitions
#define MASTER_SCK  37
#define MASTER_MISO 39
#define MASTER_MOSI 38
#define ENC1_CS     36

#define SLAVE_SCK   4
#define SLAVE_MISO  6
#define SLAVE_MOSI  5
#define ENC2_CS    1
#define ENC3_CS    2

MagneticSensorSPI encoder = MagneticSensorSPI(ENC1_CS, 14, 0x3FFF);
SPIClass hspi(HSPI);

TaskHandle_t MotorTaskHandle;
TaskHandle_t DebugTaskHandle;

// ---------------------------------------------------------
// CORE 1: Motor & Encoder Task
// ---------------------------------------------------------
void MotorTask(void *pvParameters) {
    for (;;) {
        encoder.update();
        
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

// ---------------------------------------------------------
// CORE 0: Debugging Task 
// ---------------------------------------------------------
void DebugTask(void *pvParameters) {
    bool ledState = LOW;
    
    for (;;) {
        float angle_rad = encoder.getAngle();
        float velocity = encoder.getVelocity();
        float angle_deg = angle_rad * (180.0f / PI);

        Serial.printf("Angle: %8.3f rad  |  %9.3f deg  |  Velocity: %7.3f rad/s\n",
                      angle_rad, angle_deg, velocity);

        ledState = !ledState;
        digitalWrite(43, ledState);

        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

void setup() {
    pinMode(39, INPUT);

    Serial.begin(115200);
    delay(2000); 
    Serial.println("\n--- AS5047 SPI Test Booting ---");

    pinMode(48, OUTPUT); 
    digitalWrite(48, HIGH);
    pinMode(43, OUTPUT);

    hspi.begin(MASTER_SCK, MASTER_MISO, MASTER_MOSI, ENC1_CS);
    
    encoder.init(&hspi);
    encoder.min_elapsed_time = 0.001; 

    xTaskCreatePinnedToCore(MotorTask, "MotorTask", 10000, NULL, 1, &MotorTaskHandle, 1);
    xTaskCreatePinnedToCore(DebugTask, "DebugTask", 10000, NULL, 0, &DebugTaskHandle, 0);
    
    Serial.println("FreeRTOS Tasks Started.");
}

void loop() {
    vTaskDelete(NULL); 
}