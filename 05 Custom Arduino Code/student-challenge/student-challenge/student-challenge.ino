/*
  Testing Line Tracking sensors
  Martin Hebel
  April 2023

  Test but running code and testing different reflective surfaces
  under the car's line tracking module on bottom. View in Serial Monitor.

  Develop your own code to follow the line based on sensor values
*/

// Defines to replace code to read the sensors when using LT_R, LT_M and LT_L
#define LT_R analogRead(A0)
#define LT_M analogRead(A1)
#define LT_L analogRead(A2)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // Display the values of sensors from 0 = full reflection to 1023 = no reflection
  Serial.print("Left = ");
  Serial.print(LT_L);
  Serial.print("  Middle = ");
  Serial.print(LT_M);
  Serial.print("  Right = ");
  Serial.println(LT_R);
  delay(50);
}
