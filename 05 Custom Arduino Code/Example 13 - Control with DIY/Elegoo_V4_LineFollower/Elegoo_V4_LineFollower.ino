/*
 * Elegoo Smart Robot Car V4.0 - ADC Line Follower with Built-in Start Button
 * File: Elegoo_V4_LineFollower.ino
 */

// --- Built-in Push Button (MODE button on V4 shield) ---
const int PIN_BUTTON = 2;

// --- Line Tracking Sensor ADC Pins ---
const int PIN_LT_RIGHT  = A0;
const int PIN_LT_MIDDLE = A1;
const int PIN_LT_LEFT   = A2;

// --- Motor Driver Pins (TB6612FNG on V4) ---
const int PIN_STBY = 3;  // Driver standby
const int PIN_PWMR = 5;  // Right motor PWM
const int PIN_PWML = 6;  // Left motor PWM
const int PIN_DIRR = 7;  // Right motor direction
const int PIN_DIRL = 8;  // Left motor direction

// define IO pin
#define PWMA  5    // Controls power to right motor
#define PWMB  6    // Controls power to left motor
#define AIN   7    // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define BIN   8    // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define STBY  3    // Place H-Bridge in standby if LOW, Run if HIGH

#define modeSwitch 2 // Mode Switch input


// define variables
int speedLeft = 0;  // holds the speed of left wheels
int speedRight = 0; // holds the speed of right wheels

// --- Detection Threshold ---
// White surface: typically ~30 - 180
// Black line:    typically ~500 - 850
const int LINE_THRESHOLD = 600;

// Motor speeds (PWM 0-255)
const int SPEED_FORWARD = 130;
const int SPEED_TURN    = 140;

// Function prototypes
void moveForward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stopMotors();

void setup() {
  Serial.begin(9600);

  // Configure onboard MODE button
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  // Motor pins
  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_PWML, OUTPUT);
  pinMode(PIN_PWMR, OUTPUT);
  pinMode(PIN_DIRL, OUTPUT);
  pinMode(PIN_DIRR, OUTPUT);

  // Keep motors off at power-up
  stopMotors();
  digitalWrite(PIN_STBY, HIGH);

  Serial.println(F("Elegoo V4 Line Follower Ready"));
  Serial.println(F("Place car on the line and press the built-in MODE button (D2)..."));

  // Wait for button press (active LOW)
  while (digitalRead(PIN_BUTTON) == HIGH) {
    // Stream live ADC values so students can verify surface calibration
    int r = analogRead(PIN_LT_RIGHT);
    int m = analogRead(PIN_LT_MIDDLE);
    int l = analogRead(PIN_LT_LEFT);
    Serial.print("L(A2): "); Serial.print(l);
    Serial.print(" | M(A1): "); Serial.print(m);
    Serial.print(" | R(A0): "); Serial.println(r);
    delay(1000);
  }

  
 // motorTest();



  // 1-second delay so hands are clear before moving
  delay(1000);
  Serial.println(F("Tracking started!"));
}

void loop() {
  // Read analog values from ADC (0 - 1023)
  int valL = analogRead(PIN_LT_LEFT);
  int valM = analogRead(PIN_LT_MIDDLE);
  int valR = analogRead(PIN_LT_RIGHT);

  // Determine line detection status
  bool leftDetected   = (valL > LINE_THRESHOLD);
  bool middleDetected = (valM > LINE_THRESHOLD);
  bool rightDetected  = (valR > LINE_THRESHOLD);

  printLSValues(valL,valM,valR,leftDetected,middleDetected,rightDetected);

 

  // 1. Center sensor on black line -> Straight
  if (middleDetected ) {
     Serial.println(F("Move Forward!"));
     moveForward(SPEED_FORWARD);
     return;
  }
  else if (!middleDetected) {
    stopMotors();
    return;
  }
  // 2. Left sensor detected line -> Pivot left
  else if (leftDetected && !rightDetected) {
    spinLeft(SPEED_TURN);
  }
  // 3. Right sensor detected line -> Pivot right
  else if (rightDetected && !leftDetected) {
    spinRight(SPEED_TURN);
  }
  // 4. All three sensors on black (cross intersection / stop line)
  else if (leftDetected && middleDetected && rightDetected) {
    stopMotors();
  }
  // 5. Line temporarily lost (all white) -> Coast gently forward
  else if (!leftDetected && !middleDetected && !rightDetected) {
    moveForward(90);
  }
  else {
    moveForward(SPEED_FORWARD);
  }
}




void printLSValues(int valL, int valM, int valR,bool leftDetected, bool middleDetected, bool rightDetected)
{

  Serial.print("L(A2): "); Serial.print(valL);
  Serial.print(" | M(A1): "); Serial.print(valM); 
  Serial.print(" | R(A0): "); Serial.println(valR);
  Serial.print("L(A2): "); Serial.print(leftDetected);
  Serial.print(" | M(A1): "); Serial.print(middleDetected);
  Serial.print(" | R(A0): "); Serial.println(rightDetected);
  //delay(500);

}

// --- Motor Helpers ---

void moveForward(int speed) {
  digitalWrite(PIN_DIRL, HIGH);
  analogWrite(PIN_PWML, speed);

  digitalWrite(PIN_DIRR, HIGH);
  analogWrite(PIN_PWMR, speed);
}
void moveReverse(int speed) {
  // Left Motor Backward: DIR LOW
  digitalWrite(PIN_DIRL, LOW);
  analogWrite(PIN_PWML, speed);

  // Right Motor Backward: DIR LOW
  digitalWrite(PIN_DIRR, LOW);
  analogWrite(PIN_PWMR, speed);
}
void turnLeft(int outerSpeed, int innerSpeed) {
  // Left Motor (inside wheel): DIR HIGH
  digitalWrite(PIN_DIRL, HIGH);
  analogWrite(PIN_PWML, innerSpeed);

  // Right Motor (outside wheel): DIR HIGH
  digitalWrite(PIN_DIRR, HIGH);
  analogWrite(PIN_PWMR, outerSpeed);
}

void turnRight(int outerSpeed, int innerSpeed) {
  // Left Motor (outside wheel): DIR HIGH
  digitalWrite(PIN_DIRL, HIGH);
  analogWrite(PIN_PWML, outerSpeed);

  // Right Motor (inside wheel): DIR HIGH
  digitalWrite(PIN_DIRR, HIGH);
  analogWrite(PIN_PWMR, innerSpeed);
}

void turnLeft(int speed) {
  digitalWrite(PIN_DIRL, LOW);
  analogWrite(PIN_PWML, speed);

  digitalWrite(PIN_DIRR, HIGH);
  analogWrite(PIN_PWMR, speed);
}

void turnRight(int speed) {
  digitalWrite(PIN_DIRL, HIGH);
  analogWrite(PIN_PWML, speed);

  digitalWrite(PIN_DIRR, LOW);
  analogWrite(PIN_PWMR, speed);
}

void spinLeft(int speed) {
  // Left Motor Backward: DIR LOW
  digitalWrite(PIN_DIRL, LOW);
  analogWrite(PIN_PWML, speed);

  // Right Motor Forward: DIR HIGH
  digitalWrite(PIN_DIRR, HIGH);
  analogWrite(PIN_PWMR, speed);
}

void spinRight(int speed) {
  // Left Motor Forward: DIR HIGH
  digitalWrite(PIN_DIRL, HIGH);
  analogWrite(PIN_PWML, speed);

  // Right Motor Backward: DIR LOW
  digitalWrite(PIN_DIRR, LOW);
  analogWrite(PIN_PWMR, speed);
}

void stopMotors() {
  analogWrite(PIN_PWML, 0);
  analogWrite(PIN_PWMR, 0);
}

/*

This is a second set of functions to use

*/


void waitForStart(){
  
  Serial.println(F("Waiting for Switch"));
    while (digitalRead(modeSwitch) == 1)  // wait for mode switch to be pressed (go to 0)
    ;  // Do nothing
  delay(2000);    // delay 2 seconds before starting
}











// Function stopTime - accepts the time in milliSeconds to go into standby for, returns nothing
void stopTime(int mS){
  digitalWrite(STBY, LOW);    // Go into standby
  delay(mS);                  //  Wait defined time
  digitalWrite(STBY, HIGH);   //  Come out of standby
}





void motorTest()
{

    waitForStart();             // Wait for mode button to be pressed
  
  moveForward(100);delay(1000);
  moveReverse(100);delay(1000);

  turnLeft(200,200);delay(1000);
  turnRight(200,200);delay(1000);

  stopMotors();      delay(1000);   // stop for 1 second
  spinRight(100); delay(2000);   // spin to right 3 second
  spinLeft(100);  delay(2000);   // spin to left 2 seconds
  stopMotors();      
}
