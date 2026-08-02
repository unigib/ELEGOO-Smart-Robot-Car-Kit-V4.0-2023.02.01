/*  RGB_LEDControl
    Martin Hebel, 2023
  
    Uses a communications system to send data to control the RGB LED on the Elegoo Robot 4.

    Be sure to add the FastLED driver library
    Sketch --> Include File --> Add .zip library --> FastLED_Master
*/

#include <FastLED.h>

#define NUM_LEDS 1    // Number of LEDs in strip (just 1)
#define DATA_PIN 4    // Pin for data communications
 
// Define the array of leds
CRGB leds[NUM_LEDS];

// Runs once to configure devices
void setup() { 
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // Configure the LED    
} 

// loops forever
void loop() { 
  FastLED.setBrightness(30);       // Set the brightness, 0 to 255 - Warning: 255 can be VERY bright
  
  leds[0] = CRGB::Green;            // Set Color with name - Erase Azure and press CTRL-Space for choices
  // leds[0] = CRGB(128,0,0);       // OR, Set color with amount of Red, Green and Blue, 0 to 255 each
  FastLED.show();                   // Set the LED to show
  delay(500);                       // Wait 1/2 second
  
  // Now turn the LED off, then pause
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(500);
} // repeat

