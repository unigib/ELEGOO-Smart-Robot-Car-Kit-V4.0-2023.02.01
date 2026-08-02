/*  
    Driving_with_DIY
    Martin Hebel, March 2023
    
    Demonstrates driving with the ElegooKit App, DIY Screen.

    To assign a button, hold button in app
    Enter Name of "Forward"
    Enter a Message to send (this code only uses a single character enclosed in braces)
    {F}  - with the braces for forward

    Enter a Message to send 
    {R} - with the braces for reverse

    Add more below to control more features

    Save your configuration in upper right

    Connect on WiFi with the App, ensure UPLOAD/CAM is in CAM for control,
    Press your buttons for control.

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

//init the car
void setup() {
  pinMode(PWMA, OUTPUT);     //set IO pin mode OUTPUT
  pinMode(PWMB, OUTPUT);
  pinMode(BIN, OUTPUT);
  pinMode(AIN, OUTPUT);
  pinMode(STBY, OUTPUT);

  Serial.begin(9600);
  
  analogWrite(PWMA, 0);      // Fully off for Right
  analogWrite(PWMB, 0);      // Fully off for Left
  digitalWrite(STBY, HIGH);  //Enable Motors to run
  setSpeeds(150,150);
}

//main loop to drive by calling functions for car functions based on message sent
void loop() {
  delay(100);
  Serial.println("...");
  if (Serial.available()){
    char c = Serial.read();
      switch (c) {
        case 'F':     // recieve an F, drive forward
          forward();
          delay(1000);
          stop();
          break;

        case 'R':   // revieve an R, drive reverse
          reverse();
          delay(1000);
          stop();
          break;
      }
    }
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

