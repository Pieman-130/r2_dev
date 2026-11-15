#include <Wire.h>

const int MPU_addr1 = 0x68;

const int head = 9;   //head pin D9 
const int up_limit = 220; //head highest position
const int down_limit = 130; //head lowest position
const int straight = 154; //head looking straight forward MPU6050 0deg Pitchkk

float xa, ya, za, roll, pitch;

void setup() {

  Wire.begin();                                      //begin the wire communication
  Wire.beginTransmission(MPU_addr1);                 //begin, send the slave adress (in this case 68)
  Wire.write(0x6B);                                  //make the reset (place a 0 into the 6B register)
  Wire.write(0);
  Wire.endTransmission(true);                        //end the transmission

  pinMode(head, OUTPUT);
  
  Serial.begin(9600);
}

void loop() {

  Wire.beginTransmission(MPU_addr1);
  Wire.write(0x3B);  //send starting register address
  Wire.endTransmission(false); //restart for read
  Wire.requestFrom(MPU_addr1, 6, true); //get six bytes accelerometer data

  xa = Wire.read() << 8 | Wire.read();
  ya = Wire.read() << 8 | Wire.read();
  za = Wire.read() << 8 | Wire.read();

  // formula from https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
  pitch = atan2(ya , za) * 180.0 / PI;
  roll = atan2(-xa , sqrt(ya * ya + za * za)) * 180.0 / PI; //account for roll already applied

  analogWrite(head,straight);
  //Serial.print("roll = ");
  //Serial.print(roll,1);
  Serial.print(", pitch = ");
  Serial.println(pitch,1);
  delay(400);
}