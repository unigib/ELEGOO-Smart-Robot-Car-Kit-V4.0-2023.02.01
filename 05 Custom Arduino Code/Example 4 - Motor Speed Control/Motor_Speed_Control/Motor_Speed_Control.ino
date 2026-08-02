/*  
    Motor_Speed_Control
    Martin Hebel, March 2023

    Demonstrates speed control of the TB6612FNG Dual H-Bridge Motor Driver    
    Download, unconnect cable, place on floor with room to drive (about 4 feet clearance in all directions)
    Press the mode button, wait 2 seconds for it to begin

    This example uses analogWrite(PWMA,value) where value is 0 to 255, from stopped to full speed. 
    The value needs to be a little high to produce enough torque to get it moving.

    analogWrite produces PWM, from 0% to 100% of high time per cycle, allowing from 0% to 100% power to a DC Loads
    over the range of value of 0 to 255, thus controlling motor speed
*/

// define IO pin
#define PWMA  5    // Controls power to right motor
#define PWMB  6    // Controls power to left motor
#define AIN   7    // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define BIN   8    // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define STBY  3    // Place H-Bridge in standby if LOW, Run if HIGH

#define modeSwitch 2 // Mode Switch input
//init the car
void setup() {
  pinMode(PWMA, OUTPUT);     //set IO pin mode OUTPUT
  pinMode(PWMB, OUTPUT);
  pinMode(BIN, OUTPUT);
  pinMode(AIN, OUTPUT);
  pinMode(STBY, OUTPUT);

  
  analogWrite(PWMA, 0);  // Fully off for Right
  analogWrite(PWMB, 0);  // Fully off for Left
  digitalWrite(STBY, HIGH);  //Enable Motors to run
}

//main loop
void loop() {
  while (digitalRead(modeSwitch) == 1)  // wait for mode switch to be pressed (go to 0)
    ;  // Do nothing

  delay(2000);    // delay 2 seconds before starting

  // RIGHT WHEELS 1/2 speed forward, LEFT wheels 1/4 speed forward
  // for 2 seconds
  digitalWrite(AIN, HIGH);    // Right Forward direction
  digitalWrite(BIN, HIGH);    // Left Forward direction
  analogWrite(PWMA, 128);    // 1/2 power on right
  analogWrite(PWMB, 64);     // 1/4 power on left 
  delay(2000);                //delay 2000mS

  // Switch to 1/4 power on right, 1/2 power on left
  analogWrite(PWMA, 64);   // 1/4 power on right
  analogWrite(PWMB, 128);  // 1/2 power on left 
  delay(2000);              //delay 2000mS

  // Stop both motors for 2 seconds
  analogWrite(PWMA, 0);   // no power on right
  analogWrite(PWMB, 0);   // no power on left 
  delay(2000);

 // drive progressively slower backwards  
  digitalWrite(AIN, LOW);    // Right reverse direction
  digitalWrite(BIN, LOW);    // Left reverse direction
 
  for (int speed = 255; speed > 0; speed--){  // go from 255 to 0 on speed decrementing by 1 each loop
    analogWrite(PWMA, speed);   // variable power on right
    analogWrite(PWMB, speed);   // variable power on left 
    delay(4);
  }

  stopTime(500);  // stop for 1/2 second

  // Drive progressively faster forward
  digitalWrite(AIN, HIGH);    // Right reverse direction
  digitalWrite(BIN, HIGH);    // Left reverse direction

  for (int speed = 0; speed < 256; speed++){  // go from 0 to 255 on speed incrementing by 1 each loop
    analogWrite(PWMA, speed);   // variable power on right
    analogWrite(PWMB, speed);   // variable power on left 
    delay(4);
  }

  stopTime(1000);  // stop for 1 second

  // spin in place progressively faster 
  digitalWrite(AIN, HIGH);    // Right Forward direction
  digitalWrite(BIN, LOW);    // Left reverse direction

  for (int speed = 0; speed < 256; speed++){  // go from 0 to 255 on speed incrementing by 1 each loop
    analogWrite(PWMA, speed);   // variable power on right
    analogWrite(PWMB, speed);   // variable power on left 
    delay(15);
  }

   // stop both motors
  analogWrite(PWMA, 0);   // no power on right
  analogWrite(PWMB, 0);   // no power on left
}


// Function stopTime - accepts the time in milliSeconds to go into standby for, returns nothing
void stopTime(int mS){
  digitalWrite(STBY, LOW);    // Go into standby
  delay(mS);                  //  Wait defined time
  digitalWrite(STBY, HIGH);   //  Come out of standby
}
