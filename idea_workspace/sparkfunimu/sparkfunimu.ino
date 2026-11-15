#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948

#define WIRE_PORT Wire // Your desired wire port
#define AD0_VAL 1      // The value of the last digit of the I2C address. 
                       // Address can be 0x68 or 0x69, set to 1 for 0x69 (default on many SparkFun boards)

ICM_20948_I2C myICM; 

void setup() {
  Serial.begin(115200);
  while (!Serial);

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while (!initialized) {
    myICM.begin(WIRE_PORT, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println(F("Initialization failed. Retrying..."));
      delay(500);
    } else {
      Serial.println(F("ICM-20948 connected!"));
      initialized = true;
    }
  }
}

void loop() {
  if (myICM.dataReady()) {
    myICM.getAGMT(); // The values are stored in myICM.accX, myICM.accY, myICM.accZ

    // Calculate pitch from accelerometer (in radians, then convert to degrees)
    // Pitch: rotation around Y-axis (or X depending on your board orientation)
    // Formula: atan2(accX, sqrt(accY*accY + accZ*accZ)) or similar variant
    float accX = myICM.accX();
    float accY = myICM.accY();
    float accZ = myICM.accZ();

    float pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;

    Serial.print("Pitch: ");
    Serial.println(pitch);

    delay(100);
  }
}