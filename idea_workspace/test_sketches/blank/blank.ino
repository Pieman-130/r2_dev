
const int ra = 8;
const int rb = 9; 
const int rc = 10;
const int la = 0;
const int lb = 1;
const int lc = 7;
const int lsc = 18;
const int rsc = 19;

void setup() {
    Serial.begin(115200);
    delay(1000);
    pinMode(la,INPUT);
    pinMode(lb,INPUT);
    pinMode(lc,INPUT);
    pinMode(ra,INPUT);
    pinMode(rb,INPUT);
    pinMode(rc,INPUT);
    pinMode(lsc,INPUT);
    pinMode(rsc,INPUT);
   
}

void loop() {
   //Serial.print(digitalRead(la)); Serial.print(',');
   //Serial.print(digitalRead(lb)); Serial.print(',');
   //Serial.print(digitalRead(lc)); Serial.print(',');
   Serial.print(digitalRead(rsc));
   Serial.println(' ');
   //delay(1000);



}
