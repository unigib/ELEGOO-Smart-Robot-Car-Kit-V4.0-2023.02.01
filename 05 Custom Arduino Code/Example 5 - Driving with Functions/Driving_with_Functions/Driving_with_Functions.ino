/*  
    Driving_with_Functions
    Martin Hebel, March 2023
    
    Demonstrates driving by calling functions to perform various car functions.
    Place on floor and press Mode switch to start
*/

// define IO pin
#define PWMA  5    // Controls power to right motor
#define PWMB  6    // Controls power to left motor
#define AIN   7    // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define BIN   8    // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define STBY  3    // Place H-Bridge in standby if LOW, Run if HIGH

// define variables
int speedLeft = 0;  // holds the speed of left wheels
int speedRight = 0; // holds the speed of right wheels

#define modeSwitch 2 // Mode Switch input
//init the car
void setup() {
  pinMode(PWMA, OUTPUT);     //set IO pin mode OUTPUT
  pinMode(PWMB, OUTPUT);
  pinMode(BIN, OUTPUT);
  pinMode(AIN, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  analogWrite(PWMA, 0);      // Fully off for Right
  analogWrite(PWMB, 0);      // Fully off for Left
  digitalWrite(STBY, HIGH);  //Enable Motors to run
}

//main loop to drive by calling functions for car functions
void loop() {
  waitForStart();             // Wait for mode button to be pressed
  setSpeeds(200, 200);        // sets speeds fairly high
  forward();   delay(2000);   // drive forward 2 seconds
  reverse();   delay(2000);   // drive reverse 2 seconds
  setSpeeds(100,100);         // set speeds moderate
  forward();   delay(1000);   // drive forward 1 second
  left();      delay(1000);   // turn left 1 second
  right();     delay(1000);   // turn right 1 second
  stop();      delay(1000);   // stop for 1 second
  spinRight(); delay(2000);   // spin to right 3 second
  spinLeft();  delay(2000);   // spin to left 2 seconds
  stop();                     // stop both motors and repeat
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
  digitalWrite(AIN, HIGH);
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

