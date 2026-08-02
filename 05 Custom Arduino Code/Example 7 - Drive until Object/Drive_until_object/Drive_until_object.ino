/*  Drive_until_object
    Uses Driving with Function
    Martin Hebel, March 2023
    
    Drives the car forward until the ultrasonic sensor detects an object, then turns
    
    Be sure to add the library for the sensor:
    Sketch--> Include Library --> Add Zip Library --> HC-SR04.zip

    Place on floor, press mode switch.
*/

// Configure ultrasonic sensor
#include "SR04.h"
#define TRIG_PIN 13
#define ECHO_PIN 12 
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);    // create an instance of the ultrasonic class

// define Motor IO pin
#define PWMA  5    // Controls power to right motor
#define PWMB  6    // Controls power to left motor
#define AIN   7    // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define BIN   8    // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define STBY  3    // Place H-Bridge in standby if LOW, Run if HIGH

#define modeSwitch 2 // Mode Switch input

// Define variables
int speedLeft = 0;  // holds the speed of left wheels
int speedRight = 0; // holds the speed of right wheels
long distance;      // Holds distance from sensor

//init the car
void setup() {
  pinMode(PWMA, OUTPUT);     //set IO pin mode OUTPUT
  pinMode(PWMB, OUTPUT);
  pinMode(BIN, OUTPUT);
  pinMode(AIN, OUTPUT);
  pinMode(STBY, OUTPUT);
 
  analogWrite(PWMA, 0);       // Fully off for Right
  analogWrite(PWMB, 0);       // Fully off for Left
  digitalWrite(STBY, HIGH);   //Enable Motors to run

  waitForStart();             // wait until mode is pressed
  setSpeeds(100, 100);        // sets moderate speed
}

//main loop to drive forward until an object is detected
void loop() {
  forward();                  // drive forward 
  distance=sr04.Distance();   // Get and store the distance from the device driver
  if (distance < 40) {        // if too close, stop, reverse and turn
    stop();     delay(500);
    reverse();  delay (500);
    spinLeft(); delay(250);
  }
}

void waitForStart(){
    while (digitalRead(modeSwitch) == 1)  // wait for mode switch to be pressed (go to 0)
    ;  // Do nothing
  delay(2000);    // delay 2 seconds before starting
}

void setSpeeds(int leftSpeed, int rightSpeed){    // stores speeds in variables to be used when driving
  speedLeft = leftSpeed;
  speedRight = rightSpeed;  
}

void forward(){             // drives both motors in forward direction at set speed
  digitalWrite(AIN, HIGH);
  digitalWrite(BIN, HIGH);
  analogWrite(PWMA, speedRight);
  analogWrite(PWMB, speedLeft);
}

void reverse(){             // drives both motors in reverse direction at set speed
  digitalWrite(AIN, LOW);
  digitalWrite(BIN, LOW);
  analogWrite(PWMA, speedRight);
  analogWrite(PWMB, speedLeft);
}

void stop(){                // stops drive on both motors
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0); 
}

void left(){                // turns left with 1/2 speed on left going forward
  digitalWrite(AIN, HIGH);
  digitalWrite(BIN, HIGH);
  analogWrite(PWMA, speedRight);
  analogWrite(PWMB, speedLeft / 2);
}

void right(){               // turns right with 1/2 speed on right going forward
  digitalWrite(AIN, LOW);
  digitalWrite(AIN, HIGH);
  analogWrite(PWMA, speedRight / 2);
  analogWrite(PWMB, speedLeft);    
}

void spinLeft(){            // spins left with right motor forward and left in reverse
  digitalWrite(AIN, HIGH);
  digitalWrite(BIN, LOW);
  analogWrite(PWMA, speedRight);
  analogWrite(PWMB, speedLeft);
}

void spinRight(){           // spins right with left motor going forward and right in reverse
  digitalWrite(AIN, LOW);
  digitalWrite(BIN, HIGH);
  analogWrite(PWMA, speedRight);
  analogWrite(PWMB, speedLeft);
}

// Function stopTime - accepts the time in milliSeconds to go into standby for, returns nothing
void stopTime(int mS){
  digitalWrite(STBY, LOW);    // Go into standby
  delay(mS);                  //  Wait defined time
  digitalWrite(STBY, HIGH);   //  Come out of standby
}

