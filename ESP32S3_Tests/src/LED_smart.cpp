#include <Arduino.h>
#include <FastLED.h>

// Configuration
#define LED_SMART   40          
#define NUM_LEDS    1         
#define BRIGHTNESS  100         
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB         

CRGB leds[NUM_LEDS];

void setup() {
    Serial.begin(115200);
    
    FastLED.addLeds<LED_TYPE, LED_SMART, COLOR_ORDER>(leds, NUM_LEDS)
           .setCorrection(TypicalLEDStrip);
    
    FastLED.setBrightness(BRIGHTNESS);
    Serial.println("LED Control Initialized");
}

void loop() {
    static uint8_t hue = 0;
    
    // Fill the strip with a rainbow gradient
    fill_rainbow(leds, NUM_LEDS, hue, 7);
    
    // Send the data to the LEDs
    FastLED.show();
    
    // Increment the starting hue for the next frame
    hue++;
    
    // Control the speed of the animation
    delay(20); 
}