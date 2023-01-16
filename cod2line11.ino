#include <QTRSensors.h>
const int m11Pin = 7;
const int m12Pin = 3;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

int m1Speed = 0;
int m2Speed = 0;


// increase kp’s value and see what happens
float kp = 16;
float ki = 0;
float kd = 5;


int p = 1;
int i = 0;
int d = 0;

int error = 0;
int lastError = 0;

const int maxSpeed = 255;
const int minSpeed = -200;

const int baseSpeed = 140;

QTRSensors qtr;

const int sensorCount = 6;
int sensorValues[sensorCount];
int sensors[sensorCount] = { 0, 0, 0, 0, 0, 0 };


bool isCalibrated = false;
int startSweepTime = 0;
bool didInitialSweep = false;
int sweepCount = 0;
bool isSweepingLeft = false;

int INITIAL_SWEEP_TIME = 350;
int LEFT_SWEEP_TIME = 600;
int RIGHT_SWEEP_TIME = 550;
int CALIBRATE_SPEED = 250;
int SWEEP_COUNT = 6;


void setup() {

  // pinMode setup
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, sensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode

  // // calibrate the sensor. For maximum grade the line follower should do the movement itself, without human interaction.
  // for (uint16_t i = 0; i < 400; i++) {
  //   qtr.calibrate();
  //   // do motor movement here, with millis() as to not ruin calibration)
  // }
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(9600);
}

void stopMotor() {
  for (int j = 0; j < 9000; j++) {
    goLeft(0);
  }
}

void calibrate() {
  if (isCalibrated) {
    return;
  }

  if (!didInitialSweep) {
    if (startSweepTime == 0) {
      goLeft(CALIBRATE_SPEED);
      qtr.calibrate();
      startSweepTime = millis();
    } else {
      if (millis() - startSweepTime > INITIAL_SWEEP_TIME) {
        startSweepTime = 0;
        stopMotor();
        didInitialSweep = true;
      } else {
        goLeft(CALIBRATE_SPEED);
        qtr.calibrate();
        return;
      }
    }

    return;
  }

  if (sweepCount < SWEEP_COUNT) {
    Serial.println(sweepCount);
    if (startSweepTime == 0) {
      goLeft(CALIBRATE_SPEED * (isSweepingLeft ? 1 : -1));
      qtr.calibrate();
      startSweepTime = millis();
    } else {
      int targetSweepTime = isSweepingLeft ? LEFT_SWEEP_TIME : RIGHT_SWEEP_TIME;
      if (millis() - startSweepTime > targetSweepTime) {
        startSweepTime = 0;
        isSweepingLeft = !isSweepingLeft;
        stopMotor();
        sweepCount++;
      } else {
        goLeft(CALIBRATE_SPEED * (isSweepingLeft ? 1 : -1));
        qtr.calibrate();
        return;
      }
    }

    return;
  }

  isCalibrated = true;

  // // Final half sweep from left to center
  // if (startSweepTime == 0) {
  //   goLeft(-CALIBRATE_SPEED);
  //   qtr.calibrate();
  //   startSweepTime = millis();
  // } else {
  //   if (millis() - startSweepTime > INITIAL_SWEEP_TIME) {
  //     startSweepTime = 0;
  //     stopMotor();
  //     isCalibrated = true;
  //   } else {
  //   goLeft(-CALIBRATE_SPEED);

  //     qtr.calibrate();
  //     return;
  //   }
  // }
}

void followLine() {
  // inefficient code, written in loop. You must create separate functions
  int error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);

  p = error;
  i = i + error;
  d = error - lastError;
  lastError = error;

  int motorSpeed = kp * p + ki * i + kd * d;  // = error in this case

  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  // a bit counter intuitive because of the signs
  // basically in the first if, you substract the error from m1Speed (you add the negative)
  // in the 2nd if you add the error to m2Speed (you substract the negative)
  // it's just the way the values of the sensors and/or motors lined up
  if (error < 0) {
    m1Speed += motorSpeed;
    m2Speed -= motorSpeed;
  } else if (error > 0) {
    m2Speed -= motorSpeed;
    m1Speed += motorSpeed;
  }
  // make sure it doesn't go past limits. You can use -255 instead of 0 if calibrated programmed properly.
  // making sure we don't go out of bounds
  // maybe the lower bound should be negative, instead of 0? This of what happens when making a steep turn
  m1Speed = constrain(m1Speed, minSpeed, maxSpeed);
  m2Speed = constrain(m2Speed, minSpeed, maxSpeed);


  setMotorSpeed(m1Speed, m2Speed);
}

void loop() {
  if (!isCalibrated) {
    calibrate();
  } else {
    followLine();
  }

  //  DEBUGGING
  //  Serial.print("Error: ");
  //  Serial.println(error);
  //  Serial.print("M1 speed: ");
  //  Serial.println(m1Speed);
  //
  //  Serial.print("M2 speed: ");
  //  Serial.println(m2Speed);
  //
  //  delay(250);
}

void goLeft(int speed) {
  int correctedSpeed = -speed;
  if (correctedSpeed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, correctedSpeed);
  }

  else {
    if (correctedSpeed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, correctedSpeed);
    }
    if (correctedSpeed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -correctedSpeed);
    }
  }
}


void goRight(int speed) {
  if (speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, speed);
  } else {
    if (speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, speed);
    }
    if (speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -speed);
    }
  }
}


// each arguments takes values between -255 and 255. The negative values represent the motor speed in reverse.
void setMotorSpeed(int motor1Speed, int motor2Speed) {
  // remove comment if any of the motors are going in reverse
  // motor1Speed = -motor1Speed;
  //  motor2Speed = -motor2Speed;

  goLeft(motor1Speed);
  goRight(motor2Speed);
}