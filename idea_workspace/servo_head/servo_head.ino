#include <Servo.h>
#define HEAD 9

//Servo head;
int pos = 0;

void setup() {
    //head.attach(HEAD);
    pinMode(HEAD, OUTPUT);
}

void loop() {
    /*
    for (pos=62; pos<=254; pos+=1){
        //head.write(pos);
        analogWrite(HEAD, pos);
        delay(15);
    }
    for (pos=254; pos>=62; pos-=1){
        //head.write(pos);
        analogWrite(HEAD,pos);
        delay(15);
    }
    */
   analogWrite(HEAD,160);
   //head.write(180);
}
