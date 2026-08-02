/*  
    ModeSwichReadDisplay
    Martin Hebel, March 2023
    
    Reads and displays the mode switch on the Robot Car Version 4

    Ensure car in program mode (not camera) and upload to car
    Open the Serial Monitor - Upper Right magnifying glass -  ensure set to 9600 and view data
    Press and release the button
    1 = not pressed, 0 = pressed
*/

#define modeSwitch 2     // Define pin of Mode Switch
bool modeSwitchState;   // Holds state of mode switch

// Setup runs once on reset or power-up
void setup() {
  pinMode(modeSwitch,INPUT);      // Ensure modeSwitch is an input (by deafult it should be)
  Serial.begin(9600);             // Begin serial communications at 9600 baud
}

// Loop runs forever
void loop() {
  // Serial.print() does not go to a new line, Serial.println() will go to next line after printing

  modeSwitchState = digitalRead(modeSwitch);  // read mode switch and store in modeSwitch State
  Serial.print("Mode Switch Value = ");       // Display Text for Mode Switch
  Serial.print(modeSwitchState);              // Display value of modeSwitchState in Serial Monitor

  if (modeSwitchState)  // Test if state is true (1) - braces are NOT needed if just 1 line of code for the conditional
  {                       
    Serial.println(" -- Switch not pressed");  // show this if true
  }
  else 
  {                                        
    Serial.println(" -- Switch pressed");       // show this if false
  }
}
