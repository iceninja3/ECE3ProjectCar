// driveCode.ino
// Line-following car with improved PID steering for sharp turns
#include <ECE3.h>
#include <stdio.h>

// Pin definitions (adjust pins to match your wiring)
const int sensorPins[8]   = {A0, A1, A2, A3, A4, A5, A6, A7};
const int leftMotorPWM     = 5;
const int leftMotorDir     = 4;
const int rightMotorPWM    = 6;
const int rightMotorDir    = 7;

// Calibration thresholds & PID-like gains
const int   sensorThreshold = 600;    // ADC reading threshold for black vs white
const float Kp              = 0.5;    // Proportional gain
const float Kd              = Kp * 0.1; // Derivative gain
const int   FAST            = 25;     // Speed on straight / gentle curves
const int   SLOW            = 15;     // Speed into very sharp turns
const int   turnThresh      = 300;    // Error magnitude threshold to switch to SLOW

// State variables
int lastError  = 0;
int prevError  = 0;

void setup() {
  Serial.begin(9600);
  // Initialize sensors
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  // Initialize motors
  pinMode(leftMotorPWM,  OUTPUT);
  pinMode(leftMotorDir,  OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorDir, OUTPUT);
}

void loop() {
  int raw[8];
  readSensors(raw);

  int activeCount = countActiveSensors(raw);
  int error       = computeError(raw);

  // 1) "Lost line" fallback: continue turning in the same direction
  if (activeCount == 0) {
    error = lastError;
  } else {
    lastError = error;
  }

  // 2) Derivative term
  int dTerm    = int((error - prevError) * Kd);
  prevError    = error;

  // 3) Proportional + derivative
  int delta    = int(error * Kp) + dTerm;
  // 4) Clamp to allow full pivot
  if      (delta >  FAST) delta =  FAST;
  else if (delta < -FAST) delta = -FAST;

  // 5) Slow into sharp turns
  int baseSpeed = (abs(error) > turnThresh) ? SLOW : FAST;

  // 6) Pivot overrides when only outermost sensor detects line
  if (isOnlyLeftmostActive(raw)) {
    ChangeWheelSpeeds(-baseSpeed, +baseSpeed);
  }
  else if (isOnlyRightmostActive(raw)) {
    ChangeWheelSpeeds(+baseSpeed, -baseSpeed);
  }
  else {
    // 7) Normal steering
    int leftSpeed  = baseSpeed + delta;
    int rightSpeed = baseSpeed - delta;
    ChangeWheelSpeeds(leftSpeed, rightSpeed);
  }
}

// --- Helper Functions ---

void readSensors(int raw[8]) {
  for (int i = 0; i < 8; i++) {
    int val = analogRead(sensorPins[i]);
    raw[i]  = (val > sensorThreshold) ? 1 : 0;
  }
}

int computeError(const int raw[8]) {
  // Weighted scheme: [-3, -2, -1, 0, 0, +1, +2, +3]
  const int weights[8] = {-3, -2, -1, 0, 0, 1, 2, 3};
  int num = 0, den = 0;
  for (int i = 0; i < 8; i++) {
    num += raw[i] * weights[i];
    den += raw[i];
  }
  // Return scaled error (×100) to give resolution
  return den ? (num * 100 / den) : 0;
}

int countActiveSensors(const int raw[8]) {
  int cnt = 0;
  for (int i = 0; i < 8; i++) cnt += raw[i];
  return cnt;
}

bool isOnlyLeftmostActive(const int raw[8]) {
  if (!raw[0]) return false;
  for (int i = 1; i < 8; i++) if (raw[i]) return false;
  return true;
}

bool isOnlyRightmostActive(const int raw[8]) {
  if (!raw[7]) return false;
  for (int i = 0; i < 7; i++) if (raw[i]) return false;
  return true;
}

void ChangeWheelSpeeds(int leftSpeed, int rightSpeed) {
  // Left motor control
  if (leftSpeed >= 0) {
    digitalWrite(leftMotorDir, HIGH);
    analogWrite(leftMotorPWM, map(leftSpeed, 0, FAST, 0, 255));
  } else {
    digitalWrite(leftMotorDir, LOW);
    analogWrite(leftMotorPWM, map(-leftSpeed, 0, FAST, 0, 255));
  }
  // Right motor control
  if (rightSpeed >= 0) {
    digitalWrite(rightMotorDir, HIGH);
    analogWrite(rightMotorPWM, map(rightSpeed, 0, FAST, 0, 255));
  } else {
    digitalWrite(rightMotorDir, LOW);
    analogWrite(rightMotorPWM, map(-rightSpeed, 0, FAST, 0, 255));
  }
}
