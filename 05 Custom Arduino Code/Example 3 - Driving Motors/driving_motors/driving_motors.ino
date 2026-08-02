/*  driving_motors
    Martin Hebel, 2023
  
    Code to test the motors on the Elegoo Robot Car 4  

    Download, disconnect cable, place on floor with room to drive  
    (about 4 feet clearance in all directions)
    Press the mode button, wait 2 seconds for it to begin
*/

// define IO pin
#define PWMA  5     // Controls power to right motor
#define PWMB  6     // Controls power to left motor
#define AIN   7     // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define BIN   8     // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define STBY  3     // Place H-Bridge in standby if LOW, Run if HIGH

#define modeSwitch 2 // Mode Switch input
//initialize the car
void setup() {
  pinMode(PWMA, OUTPUT);     //set IO pin mode OUTPUT
  pinMode(PWMB, OUTPUT);
  pinMode(BIN, OUTPUT);
  pinMode(AIN, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(PWMA, LOW);   // Fully Off
  digitalWrite(STBY, HIGH);  //Enable Motors to run
   
}

//main loop
void loop() {
  while (digitalRead(modeSwitch) == 1)  // wait for mode switch to be pressed (go to 0)
    ;  // Do nothing

  delay(2000);

  // RIGHT WHEELS
  // Drive Right Wheels forward for 2 second  
  digitalWrite(AIN, HIGH);    // Forward direction
  digitalWrite(PWMA, HIGH);   // Full power
  delay(2000);                //delay 2000mS

  // Stop for 2 seconds
  stopTime(2000);

  // Drive Right Wheels Backwards for 2 second  
  digitalWrite(AIN, LOW);    // Backwards direction
  digitalWrite(PWMA, HIGH);  // Full power
  delay(2000);               //delay 2000mS

  // Stop for 2 seconds
  stopTime(2000);

  // Turn off Right Motor's Power
  digitalWrite(PWMA, LOW);

  // DRIVE LEFT WHEELS
  // Drive LEFT Wheels forward for 2 second  
  digitalWrite(BIN, HIGH);    // Forward direction
  digitalWrite(PWMB, HIGH);   // Full power
  delay(2000);              //delay 2000mS

  // Stop for 2 seconds
  stopTime(2000);

  // Drive Left Wheels Backwards for 2 second  
  digitalWrite(BIN, LOW);    // Forward direction
  digitalWrite(PWMB, HIGH);  // Full power
  delay(2000);               //delay 2000mS

  // Stop for 2 seconds
  stopTime(2000);

  // Drive car Forward for 1 second
  digitalWrite(AIN, HIGH);    // Forward direction on Right
  digitalWrite(BIN, HIGH);    // Forward direction on Left
  digitalWrite(PWMA, HIGH);   // Full power on Right
  digitalWrite(PWMB, HIGH);   // Full power on Left
  delay(1000);              //delay 2000mS

  // stop for 2 seconds
  stopTime(2000);

  // Drive car Backwards for 1 second1
  digitalWrite(AIN, LOW);    // Reverse direction on Right
  digitalWrite(BIN, LOW);    // Reverse direction on Left
  delay(1000);

  // stop for 2 seconds
  stopTime(2000);
  
  // All motor power off
  digitalWrite(PWMA, LOW);   // No power on Right
  digitalWrite(PWMB, LOW);   // No power on Left
}

// Function - accepts the time in milli-Seconds to go into standby for
void stopTime(int mS){
  digitalWrite(STBY, LOW);   // Go into standby
  delay(mS);                //  Wait defined time
  digitalWrite(STBY, HIGH);  //  Come out of standby
}
