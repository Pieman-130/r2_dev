#include <Servo.h>

#define LEFTMOTOR 9
#define RIGHTMOTOR 10
#define LEFTDIR 5
#define RIGHTDIR 4
#define PWHIGH 10000
#define PWLOW 100
#define FWD 1
#define REV 0

Servo left;
Servo right;

void setup() {
    pinMode(LEFTDIR,OUTPUT);
    digitalWrite(LEFTDIR,LOW); //High is forward for left motor
    //left.attach(LEFTMOTOR,PWLOW,PWHIGH);
    //left.write(0);
    
    pinMode(RIGHTDIR,OUTPUT);
    digitalWrite(RIGHTDIR,LOW); //LOW is forward for right motor
    //right.attach(RIGHTMOTOR,PWLOW,PWHIGH);
    //right.write(0);


}

void loop() {
    analogWrite(LEFTMOTOR, 20);
    //delay(5000);
    //right.write(90);
    //left.write(90);
    delay(5000);
    analogWrite(LEFTMOTOR,0);
    //right.write(0);
    //left.write(0);
    digitalWrite(LEFTDIR,LOW);
    digitalWrite(RIGHTDIR,HIGH);
    delay(5000);
    //right.write(90);
    //left.write(90);
    delay(5000);
    //right.write(0);
    //left.write(0);
    while(true);

}

