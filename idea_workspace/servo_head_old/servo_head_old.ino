#include <Servo.h>
#define HEAD 9

//Servo head;
int pos = 0;

void setup() {
    //head.attach(HEAD);
    pinMode(HEAD, OUTPUT);
}

void loop() {
    
    for (pos=0; pos<=110; pos+=1){
        //head.write(pos);
        analogWrite(HEAD, pos);
        delay(50);
    }
    delay(5000);

    for (pos=110; pos>=0; pos-=1){
        //head.write(pos);
        analogWrite(HEAD,pos);
        delay(50);
    }
    delay(5000);
    
   //analogWrite(HEAD,160);
   //head.write(180);
}
