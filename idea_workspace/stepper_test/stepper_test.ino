#include <AccelStepper.h>

AccelStepper stepper(1, 13, 12); 

void setup() {  
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(1000);
  stepper.moveTo(0); 
  //stepper.setSpeed(100);
}

void loop() {
  if (stepper.distanceToGo() == 0) {
    delay(100);
    stepper.moveTo(-stepper.currentPosition());
    delay(100);
  }
  stepper.run();
  //stepper.runSpeed();
}
