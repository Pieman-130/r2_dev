
#include <Wire.h>
#include "ICM_20948.h"

#define WIRE_PORT Wire
#define AD0_VAL 1   // Default SparkFun breakout has AD0 pulled high (address 0x69). Set to 0 if you've jumpered it low (0x68).

ICM_20948_I2C myICM;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Leonardo's native USB serial

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while (!initialized) {
    myICM.begin(WIRE_PORT, AD0_VAL);

    Serial.print(F("Initialization status: "));
    Serial.println(myICM.statusString());

    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println(F("Trying again..."));
      delay(500);
    } else {
      initialized = true;
    }
  }

  Serial.println(F("ICM-20948 connected!"));
}

void loop() {
  if (myICM.dataReady()) {
    myICM.getAGMT(); // reads accel, gyro, mag, temp into myICM.agmt

    Serial.print("Acc (mg) [");
    Serial.print(myICM.accX(), 1); Serial.print(", ");
    Serial.print(myICM.accY(), 1); Serial.print(", ");
    Serial.print(myICM.accZ(), 1);
    Serial.print("]  Gyro (DPS) [");
    Serial.print(myICM.gyrX(), 1); Serial.print(", ");
    Serial.print(myICM.gyrY(), 1); Serial.print(", ");
    Serial.print(myICM.gyrZ(), 1);
    Serial.print("]  Mag (uT) [");
    Serial.print(myICM.magX(), 1); Serial.print(", ");
    Serial.print(myICM.magY(), 1); Serial.print(", ");
    Serial.print(myICM.magZ(), 1);
    Serial.print("]  Temp (C) ");
    Serial.println(myICM.temp(), 1);

    delay(50);
  }
}