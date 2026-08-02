/*  Code to demonstrate servo motion
    Modified from Elegoo's Super Starter Kit Code
    Martin Hebel, March 2023

    www.elegoo.com
    2018.12.19

    Be sure to add the library, Servo.zip.
    Sketch--> Include Library --> Add Zip Library --> Servo.zip
*/
#include <Servo.h>
/* After including the corresponding libraries,
   we can use the "class" like "Servo" created by the developer for us.
   We can use the functions and variables created in the libraries by creating 
   objects like the following "myservo" to refer to the members in ".".*/

Servo servo1;  // note that the board has 2 servo ports
//it created an object called myservo.
/*  you can see Servo as a complex date type(Including functions and various data types)
    and see myservo as variables.               */

void setup(){
  /*"attach" and "write" are both functions,
     and they are members contained in the complex structure of "Servo". 
     We can only use them if we create the object "myservo" for the complex structure of "Servo".
  */
  servo1.attach(10);   //connect pin 10 with the control line of the servo
  servo1.write(90);    // move servos to center position -> 90°
} 
void loop(){
  servo1.write(90);// move servos to center position -> 90°
  delay(1000);
  servo1.write(30);// move servos position -> 30°
  delay(1000);
  servo1.write(90);// move servos to center position -> 90°
  delay(1000);
  servo1.write(150);// move servos position -> 120°
  delay(1000);
}
