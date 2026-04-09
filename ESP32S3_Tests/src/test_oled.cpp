#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// I2C Pins
#define MASTER_SDA 4
#define MASTER_SCL 5
#define OLED_RESET -1 

// LED Pin
#define LED_PIN 43

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
    Serial.begin(115200);
    
    // Configure LED
    pinMode(LED_PIN, OUTPUT);

    // Initialise I2C
    Wire.begin(MASTER_SDA, MASTER_SCL);
    Wire.setClock(100000); 

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
        Serial.println(F("SSD1306 allocation failed"));
        // Rapid blink means ERROR
        for(int i=0; i<10; i++) {
            digitalWrite(LED_PIN, HIGH); delay(100);
            digitalWrite(LED_PIN, LOW);  delay(100);
        }
        for(;;); 
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("OLED + LED Test");
    display.display();
}

void loop() {
    // Heartbeat LED
    digitalWrite(LED_PIN, HIGH);
    
    // Update OLED
    display.clearDisplay();
    display.setCursor(0, 10);
    display.setTextSize(2);
    display.printf("Tick: %lu", millis() / 1000);
    display.display();
    
    delay(500);
    
    digitalWrite(LED_PIN, LOW);
    delay(500);
}