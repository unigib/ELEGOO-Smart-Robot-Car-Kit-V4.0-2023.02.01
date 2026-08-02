/*  
    Obstacle_avoidance
    Martin Hebel, March 2023

    Drives until an obstacle is noted, looks around, then turns towards most open area. 
    Based on code from Elegoo Robot Car V3
    
    Be sure to add the library for the devices:
    Sketch--> Include Library --> Add Zip Library --> HC-SR04.zip and servo.zip

    Place on floor, press mode switch, drives forward until too close to object, 
    figures out best way to go.

    Press mode switch to start.
*/

// Ultrasonic configuration
#include "SR04.h"
#define TRIG_PIN 13
#define ECHO_PIN 12 
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);    // create an instance of the ultrasonic class

// Servo configuration
#include <Servo.h>
#define servoPin 10
Servo servo1;                           // create an instance of the servo class

// define Motor IO pin
#define PWMA  5    // Controls power to right motor
#define PWMB  6    // Controls power to left motor
#define AIN   7    // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define BIN   8    // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define STBY  3    // Place H-Bridge in standby if LOW, Run if HIGH

// variables
int speedLeft = 0;  // holds the speed of left wheels
int speedRight = 0; // holds the speed of right wheels
int rightDistance = 0, leftDistance = 0, middleDistance = 0;  // holds distances when it 'looks around'

#define modeSwitch 2 // Mode Switch input

//initilize the car
void setup() {
  pinMode(PWMA, OUTPUT);     //set IO pin mode OUTPUT
  pinMode(PWMB, OUTPUT);
  pinMode(BIN, OUTPUT);
  pinMode(AIN, OUTPUT);
  pinMode(STBY, OUTPUT);
 
  analogWrite(PWMA, 0);       // Fully off for Right
  analogWrite(PWMB, 0);       // Fully off for Left
  digitalWrite(STBY, HIGH);  //Enable Motors to run

  servo1.attach(servoPin);    //connect pin 10 with the control line of the servo 
  servo1.write(90);           // move servos to center position -> 90°

  waitForStart();             // wait until mode is pressed
  setSpeeds(100, 100);        // sets speeds fairly high
}

//main loop to drive forward until an object is detected
void loop() {
    servo1.write(90);                   //Look forward
    middleDistance = sr04.Distance();   // Get and store the distance from the device driver

    if(middleDistance <= 30) {          // if nearby obstacle in front...
      stop();
      delay(250);
      servo1.write(10);                  // look right
      delay(1000);
      rightDistance = sr04.Distance();   // Get and store the distance from the device driver

      servo1.write(180);                // look left
      delay(1000);
      leftDistance = sr04.Distance();   // Get and store the distance from the device driver

      servo1.write(90);                 // look forward
      if((rightDistance <= 30) || (leftDistance <= 30)) {  // if both side too close
        reverse();
        delay(500);
        spinLeft();
        delay(180);
      }
      else if(rightDistance > leftDistance) {   // if right more open
        spinRight();
        delay(180);
      }
      else if(rightDistance < leftDistance) {  // if left more open
        spinLeft();
        delay(180);
      }
      else {
        forward();
      }
    }
    else {
        forward();
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

