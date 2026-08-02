/* 
  Ultrasonic_Measurement
  Modified from Elegoo's SR04_Example.ino in the SuperStarter Kit.
  Martin Hebel, March 2023

  Demonstrates operation of the ultrasonic sensor.

  Be sure to add the library for the sensor:
  Sketch--> Include Library --> Add Zip Library --> HC-SR04.zip

  Open the Serial Monitor (upper right) and monitor the distance it "sees"
*/

// Configure ultrasonic sensor
#include "SR04.h"
#define TRIG_PIN 13
#define ECHO_PIN 12
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);

// Define variables
long distance;

void setup() {
   Serial.begin(9600);//Initialization of Serial Port
   delay(500);
}

void loop() {
   distance=sr04.Distance();  // Get and store the distance from the device driver
   Serial.print(distance);
   Serial.println("cm");//The difference between "Serial.print" and "Serial.println" 
                        //is that "Serial.println" can change lines.
   delay(250);
}
