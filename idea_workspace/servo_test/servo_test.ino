#include <Servo.h>

Servo myServo;  // Create a servo object

void setup() {
  myServo.attach(10);  // Attach the servo on pin 9
}

void loop() {

  //myServo.write(90);   // Move to 0 degrees
  //delay(1000);        // Wait 1 second
  
  //myServo.write(90);  // Move to 90 degrees (middle)
  //delay(1000);        // Wait 1 second
  
  //myServo.write(90); // Move to 180 degrees
  //delay(1000);        // Wait 1 second
  
  myServo.write(90); // Move to 180 degrees
  delay(1000);        // Wait 1 second 
  myServo.detach();
 //myServo.write(0);
}