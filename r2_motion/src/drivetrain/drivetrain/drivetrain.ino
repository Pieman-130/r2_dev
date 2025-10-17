#include <ArduinoJson.h>

#define LEFTMOTOR 9
#define RIGHTMOTOR 10
#define LEFTDIR 5
#define RIGHTDIR 4

void setup() {
    pinMode(LEFTDIR,OUTPUT);
    digitalWrite(LEFTDIR,LOW); //High is forward for left motor
    
    pinMode(RIGHTDIR,OUTPUT);
    digitalWrite(RIGHTDIR,LOW); //LOW is forward for right motor

}

void loop() {
    analogWrite(LEFTMOTOR, 0);
    analogWrite(RIGHTMOTOR,0);
    
    while(true);

}

