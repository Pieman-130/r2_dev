/*
 * Two-Wheel Robot Motor Controller V1
 *
 * Hardware:
 *   Left  PWM: pin 9
 *   Left  DIR: pin 5
 *   Right PWM: pin 10
 *   Right DIR: pin 4
 *
 * Serial packet:
 *
 *   +------+------+-----+-----+------+------+-----+
 *   | 0xAA | 0x55 | LEN | CMD | LEFT | RIGHT| CRC |
 *   +------+------+-----+-----+------+------+-----+
 *
 * LEN = number of bytes from CMD through the end of DATA
 *
 * CMD 0x01 = Set motor speeds
 *
 * LEFT / RIGHT:
 *   -100 = full reverse
 *      0 = stop
 *   +100 = full forward
 *
 * V1 safety limits:
 *   Maximum PWM = 20%
 *   Acceleration/deceleration limited
 *   250 ms communication watchdog
 */

#include <Arduino.h>

// --------------------------------------------------
// Pin definitions
// --------------------------------------------------

const uint8_t LEFT_PWM_PIN  = 9;
const uint8_t LEFT_DIR_PIN  = 5;

const uint8_t RIGHT_PWM_PIN = 10;
const uint8_t RIGHT_DIR_PIN = 4;


// --------------------------------------------------
// Safety / motion parameters
// --------------------------------------------------

// Maximum physical PWM output during initial testing.
// 255 = 100%
// 51  = 20%
const uint8_t MAX_PWM = 51;

// Motor command is updated this many times per second.
const uint16_t MOTOR_UPDATE_MS = 10;

// Maximum change in PWM per motor update.
//
// Example:
//   2 PWM counts every 10 ms
//   = 200 PWM counts/sec
//
// We'll start conservatively.
const uint8_t PWM_STEP = 1;

// Stop motors if we haven't received a valid command
// within this many milliseconds.
const unsigned long COMMAND_TIMEOUT_MS = 250;


// --------------------------------------------------
// Command state
// --------------------------------------------------

int8_t requestedLeft  = 0;
int8_t requestedRight = 0;

int16_t actualLeftPWM  = 0;
int16_t actualRightPWM = 0;

unsigned long lastValidCommand = 0;
unsigned long lastMotorUpdate = 0;


// --------------------------------------------------
// CRC-8
// Polynomial: 0x07
// --------------------------------------------------

uint8_t crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0x00;

  for (uint8_t i = 0; i < length; i++)
  {
    crc ^= data[i];

    for (uint8_t bit = 0; bit < 8; bit++)
    {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }

  return crc;
}


// --------------------------------------------------
// Convert -100..100 motor command to PWM magnitude
// --------------------------------------------------

uint8_t commandToPWM(int8_t command)
{
  int value = abs(command);

  if (value > 100)
    value = 100;

  return map(value, 0, 100, 0, MAX_PWM);
}


// --------------------------------------------------
// Apply motor output
// --------------------------------------------------

void writeMotor(
  uint8_t pwmPin,
  uint8_t dirPin,
  int16_t pwmValue,
  bool reverseDirection
)
{
  if (pwmValue == 0)
  {
    analogWrite(pwmPin, 0);
    return;
  }

  bool forward = pwmValue > 0;

  if (reverseDirection)
    forward = !forward;

  digitalWrite(dirPin, forward ? HIGH : LOW);

  analogWrite(pwmPin, abs(pwmValue));
}


// --------------------------------------------------
// Update one motor toward requested speed
// --------------------------------------------------

int16_t updateMotorPWM(
  int16_t currentPWM,
  int8_t requestedCommand
)
{
  int16_t targetPWM = commandToPWM(requestedCommand);

  if (requestedCommand < 0)
    targetPWM = -targetPWM;

  // Accelerate/decelerate toward target
  if (currentPWM < targetPWM)
  {
    currentPWM += PWM_STEP;

    if (currentPWM > targetPWM)
      currentPWM = targetPWM;
  }
  else if (currentPWM > targetPWM)
  {
    currentPWM -= PWM_STEP;

    if (currentPWM < targetPWM)
      currentPWM = targetPWM;
  }

  return currentPWM;
}


// --------------------------------------------------
// Emergency motor stop
// --------------------------------------------------

void stopMotors()
{
  requestedLeft = 0;
  requestedRight = 0;

  actualLeftPWM = 0;
  actualRightPWM = 0;

  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
}


// --------------------------------------------------
// Process valid motor command
// --------------------------------------------------

void processMotorCommand(int8_t left, int8_t right)
{
  requestedLeft = constrain(left, -100, 100);
  requestedRight = constrain(right, -100, 100);

  lastValidCommand = millis();
}


// --------------------------------------------------
// Serial packet parser
// --------------------------------------------------

enum ParserState
{
  WAIT_PREAMBLE_1,
  WAIT_PREAMBLE_2,
  WAIT_LENGTH,
  RECEIVE_PACKET
};

ParserState parserState = WAIT_PREAMBLE_1;

uint8_t packetLength = 0;
uint8_t packetIndex = 0;

uint8_t packetBuffer[16];

void processSerial()
{
  while (Serial.available() > 0)
  {
    uint8_t byteReceived = Serial.read();

    switch (parserState)
    {
      case WAIT_PREAMBLE_1:

        if (byteReceived == 0xAA)
          parserState = WAIT_PREAMBLE_2;

        break;


      case WAIT_PREAMBLE_2:

        if (byteReceived == 0x55)
        {
          parserState = WAIT_LENGTH;
        }
        else if (byteReceived == 0xAA)
        {
          // Stay here. This could be the beginning
          // of another preamble.
          parserState = WAIT_PREAMBLE_2;
        }
        else
        {
          parserState = WAIT_PREAMBLE_1;
        }

        break;


      case WAIT_LENGTH:

        packetLength = byteReceived;

        // Prevent malformed packets from overflowing
        // our buffer.
        if (packetLength == 0 || packetLength > sizeof(packetBuffer))
        {
          parserState = WAIT_PREAMBLE_1;
        }
        else
        {
          packetIndex = 0;
          parserState = RECEIVE_PACKET;
        }

        break;


      case RECEIVE_PACKET:

        packetBuffer[packetIndex++] = byteReceived;

        if (packetIndex >= packetLength)
        {
          /*
           * Packet contents:
           *
           * packetBuffer[0] = CMD
           * packetBuffer[1...] = DATA
           *
           * Last byte is CRC.
           */

          if (packetLength >= 2)
          {
            uint8_t receivedCRC =
              packetBuffer[packetLength - 1];

            uint8_t calculatedCRC =
              crc8(packetBuffer, packetLength - 1);

            if (receivedCRC == calculatedCRC)
            {
              uint8_t command = packetBuffer[0];

              if (command == 0x01 &&
                  packetLength == 4)
              {
                int8_t left =
                  (int8_t)packetBuffer[1];

                int8_t right =
                  (int8_t)packetBuffer[2];

                processMotorCommand(left, right);
              }
            }
          }

          parserState = WAIT_PREAMBLE_1;
        }

        break;
    }
  }
}


// --------------------------------------------------
// Update motors
// --------------------------------------------------

void updateMotors()
{
  unsigned long now = millis();

  if (now - lastMotorUpdate < MOTOR_UPDATE_MS)
    return;

  lastMotorUpdate = now;

  // Watchdog
  if (now - lastValidCommand > COMMAND_TIMEOUT_MS)
  {
    requestedLeft = 0;
    requestedRight = 0;
  }

  actualLeftPWM =
    updateMotorPWM(actualLeftPWM, requestedLeft);

  actualRightPWM =
    updateMotorPWM(actualRightPWM, requestedRight);

  /*
   * LEFT motor:
   *
   * Software + = robot forward
   *
   * RIGHT motor:
   *
   * Software + = robot forward
   *
   * Because the motors are physically mirrored,
   * the right motor direction is inverted here.
   */

  writeMotor(
    LEFT_PWM_PIN,
    LEFT_DIR_PIN,
    actualLeftPWM,
    false
  );

  writeMotor(
    RIGHT_PWM_PIN,
    RIGHT_DIR_PIN,
    actualRightPWM,
    true
  );
}


// --------------------------------------------------
// Setup
// --------------------------------------------------

void setup()
{
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);

  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  stopMotors();

  Serial.begin(115200);

  lastValidCommand = millis();
}


// --------------------------------------------------
// Main loop
// --------------------------------------------------

void loop()
{
  processSerial();
  updateMotors();
}