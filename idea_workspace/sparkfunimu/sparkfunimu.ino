
#include <Wire.h>
#include <ICM_20948.h>   // SparkFun ICM-20948 Arduino library

ICM_20948_I2C myICM;

unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL_MS = 100;  // 10 Hz, readable on the monitor

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(400000);

  bool ok = false;
  while (!ok) {
    myICM.begin(Wire, 1);  // 1 = AD0 pulled high; use 0 if AD0 is low
    if (myICM.status == ICM_20948_Stat_Ok) {
      ok = true;
    } else {
      Serial.println("IMU init failed, retrying...");
      delay(500);
    }
  }

  Serial.println("IMU test ready.");
  Serial.println("Tilt the robot by hand and watch which values move.");
  Serial.println("accelPitch/accelRoll use accel only (noisy but no drift).");
  Serial.println();
}

void loop() {
  if (myICM.dataReady()) {
    myICM.getAGMT();

    if (millis() - lastPrint >= PRINT_INTERVAL_MS) {
      lastPrint = millis();

      float ax = myICM.accX();
      float ay = myICM.accY();
      float az = myICM.accZ();
      float gx = myICM.gyrX();
      float gy = myICM.gyrY();
      float gz = myICM.gyrZ();

      // Two independent tilt estimates from accel alone — use these to
      // figure out which axis pair corresponds to "falling forward/back"
      float pitchFromXZ = atan2(ax, az) * 180.0 / PI;
      float pitchFromYZ = atan2(ay, az) * 180.0 / PI;

      Serial.print("accX="); Serial.println(ax, 2);
      Serial.print(" accY="); Serial.print(ay, 2);
      Serial.print(" accZ="); Serial.print(az, 2);
      Serial.print(" | gyrX="); Serial.print(gx, 2);
      Serial.print(" gyrY=");   Serial.print(gy, 2);
     Serial.print(" gyrZ=");   Serial.print(gz, 2);
      Serial.print(" | pitch(x,z)="); Serial.print(pitchFromXZ, 1);
      Serial.print(" pitch(y,z)=");   Serial.println(pitchFromYZ, 1);
    }
  }
}