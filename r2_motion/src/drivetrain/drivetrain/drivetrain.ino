#define LEFTMOTOR 9     // PWM pin for left motor
#define RIGHTMOTOR 10    // PWM pin for right motor
#define LEFTDIR 5       // Direction pin for left motor
#define RIGHTDIR 4      // Direction pin for right motor

const byte PREAMBLE = 0xAA;
const unsigned long FAILSAFE_TIMEOUT = 1000; // milliseconds (1 second)
unsigned long lastCommandTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(LEFTMOTOR, OUTPUT);
  pinMode(RIGHTMOTOR, OUTPUT);
  pinMode(LEFTDIR, OUTPUT);
  pinMode(RIGHTDIR, OUTPUT);

  stopMotors();
}

void loop() {
  if (Serial.available() >= 6) {
    byte preamble = Serial.read();
    byte leftDir = Serial.read();
    byte leftSpeed = Serial.read();
    byte rightDir = Serial.read();
    byte rightSpeed = Serial.read();
    byte checksum = Serial.read();

    byte calcChecksum = (preamble + leftDir + leftSpeed + rightDir + rightSpeed) & 0xFF;

    if (preamble == PREAMBLE && checksum == calcChecksum) {
      controlMotors(leftDir, leftSpeed, rightDir, rightSpeed);
      lastCommandTime = millis(); // reset failsafe timer
    } else {
      Serial.println("Checksum error");
      flushSerial();
    }
  }

  // Failsafe check — stop if no command recently
  if (millis() - lastCommandTime > FAILSAFE_TIMEOUT) {
    stopMotors();
  }
}

void controlMotors(byte leftDir, byte leftSpeed, byte rightDir, byte rightSpeed) {
  // Set direction — note the reversed logic for right motor
  digitalWrite(LEFTDIR, leftDir ? HIGH : LOW);
  digitalWrite(RIGHTDIR, rightDir ? LOW : HIGH);

  // Set speed (PWM)
  analogWrite(LEFTMOTOR, leftSpeed);
  analogWrite(RIGHTMOTOR, rightSpeed);
}

void stopMotors() {
  analogWrite(LEFTMOTOR, 0);
  analogWrite(RIGHTMOTOR, 0);
}

void flushSerial() {
  while (Serial.available()) Serial.read();
}

//arduino-cli compile --fqbn arduino:avr:leonardo
//arduino-cli upload -p /dev/ttyACM0  --fqbn arduino:avr:leonardo