
#define LEFTMOTOR 9     // PWM pin for left motor
#define RIGHTMOTOR 10   // PWM pin for right motor
#define LEFTDIR 5       // Direction pin for left motor
#define RIGHTDIR 4      // Direction pin for right motor

void setup() {
  Serial.begin(9600);
  pinMode(LEFTMOTOR, OUTPUT);
  pinMode(RIGHTMOTOR, OUTPUT);
  pinMode(LEFTDIR, OUTPUT);
  pinMode(RIGHTDIR, OUTPUT);
  stopMotors();

  Serial.println("Motor test ready.");
  printHelp();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    char which = cmd.charAt(0);
    int value = cmd.substring(1).toInt();  // signed: negative = reverse

    switch (which) {
      case 'l': case 'L':
        driveMotor(LEFTMOTOR, LEFTDIR, value, false);
        Serial.print("LEFT -> "); Serial.println(value);
        break;
      case 'r': case 'R':
        driveMotor(RIGHTMOTOR, RIGHTDIR, value, true);
        Serial.print("RIGHT -> "); Serial.println(value);
        break;
      case 's': case 'S':
        stopMotors();
        Serial.println("STOPPED");
        break;
      case 'h': case 'H':
        printHelp();
        break;
      default:
        Serial.println("Unrecognized command.");
        printHelp();
    }
  }
}

void driveMotor(int pwmPin, int dirPin, int value, bool reversedDir) {
  bool forward = value >= 0;
  bool dirHigh = reversedDir ? !forward : forward;
  digitalWrite(dirPin, dirHigh ? HIGH : LOW);
  analogWrite(pwmPin, constrain(abs(value), 0, 255));
}

void stopMotors() {
  analogWrite(LEFTMOTOR, 0);
  analogWrite(RIGHTMOTOR, 0);
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  L<value>  e.g. L200 or L-200  (left motor, dir+speed)");
  Serial.println("  R<value>  e.g. R150 or R-150  (right motor, dir+speed)");
  Serial.println("  S         stop both motors");
  Serial.println("  H         show this help");
}