/*  Martin Hebel
    Adapted from Elegoo Super Starter Code

    IR REMOTE for Elegoo Robot Car 4

    Install IRremote.zip as a zip library

    Upload and press an IR remote button, use arrow keys at top to drive car, OK to stop.
    monitor key presses in Serial Monitor.
    */

#include "IRremote.h"

// define IO pin
#define PWMA  5    // Controls power to right motor
#define PWMB  6    // Controls power to left motor
#define AIN   7    // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define BIN   8    // Controls direction of right motor, HIGH = FORWARD, LOW = REVERSE
#define STBY  3    // Place H-Bridge in standby if LOW, Run if HIGH

IRrecv irrecv(9);             // create instance of 'irrecv'
decode_results results;      // create instance of 'decode_results'

// define variables
int speedLeft = 0;  // holds the speed of left wheels
int speedRight = 0; // holds the speed of right wheels
char lastKey = ' ';           // store last pressed for repeating key actions

void setup() {
  Serial.begin(9600);
  Serial.println("IR Receiver Button Decode");
  irrecv.enableIRIn();
  digitalWrite(STBY, HIGH);  //Enable Motors to run
  setSpeeds(200,200);        // set speeds
}

void loop()
{
  int tmpValue;
  if (irrecv.decode(&results)) // have we received an IR signal?
  {
    Serial.print(results.value, HEX); // Display key code
    Serial.print("   ");
    switch(results.value)
      {   // each code corresponds to a button on the remote.
      case 0xFF629D: 
        Serial.println("Forward"); 
        lastKey = 'f';    // store last button for repeat code when held
        forward();
        break;
      
      case 0xFF22DD: 
        Serial.println("LEFT");  
        lastKey = 'l';
        left();
        delay(500);
        forward();         
        break;

      case 0xFF02FD: 
        Serial.println("STOP");
        lastKey = 's';  
        stop();   
        break;

      case 0xFFC23D: 
        Serial.println("Right");
        lastKey = 'r';
        right(); 
        delay(500);
        forward();      
        break;

      case 0xFFA857: 
        Serial.println("BACK");
        reverse();
        lastKey = 'b';     
        break;
      

      case 0xFF6897: 
        Serial.println("1");  
        lastKey = '1';   
        break;

      case 0xFF9867: 
        Serial.println("2");
        lastKey = '2';    
        break;

      case 0xFFB04F: 
        Serial.println("3");
        lastKey = '3'; 
        break;

      case 0xFF30CF: 
        Serial.println("4");
        lastKey = '4';    
        break;

      case 0xFF18E7: 
        Serial.println("5");
        lastKey = '5';    
        break;

      case 0xFF7A85: 
        Serial.println("6");
        lastKey = '6';    
        break;

      case 0xFF10EF: 
        Serial.println("7");
        lastKey = '7';    
        break;

      case 0xFF38C7: 
        Serial.println("8");
        lastKey = '8';    
        break;

      case 0xFF5AA5: 
        Serial.println("9");
        lastKey = '9';    
        break;

      case 0xFF42BD: 
        Serial.println("*");
        lastKey = '*';    
        break;
      
      case 0xFF4AB5: 
        Serial.println("0");    
        lastKey = '0';
        break;
        
      case 0xFF52AD: 
        Serial.println("#");    
        lastKey = '#';
        break;

      case 0xFFFFFFFF: 
          Serial.print("Repeat: ");
          Serial.println(lastKey);
          if (lastKey == 'l'){    // repeat left turns if held
            left();
            delay(500);
            forward();
           }  
          break;

          if (lastKey == 'r'){    // repeat right turns if held
            right();
            delay(500);
            forward();
           }  
          break;


      default: 
        Serial.println("?");
        lastKey = ' ';
        break;
      }// End Case
  
  irrecv.resume(); // receive the next value
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


