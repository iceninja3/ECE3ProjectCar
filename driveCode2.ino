int lastError = 0;
int prevError = 0;
const int FAST       = 25;
const int SLOW       = 15;
const int turnThresh = 300;  // tune this based on your scaled error range

void loop() {
  readSensors(sensorValues);
  int rawSum = countActiveSensors(sensorValues);
  int error  = computeError(sensorValues);

  // 1) Fallback when you lose the line
  if (rawSum == 0) {
    error = lastError;
  } else {
    lastError = error;
  }

  // 2) Optional D-term
  int dTerm = (error - prevError) * Kd;
  prevError = error;

  // 3) Compute delta and saturate
  int delta = error * Kp + dTerm;
  if      (delta >  baseSpeed) delta =  baseSpeed;
  else if (delta < -baseSpeed) delta = -baseSpeed;

  // 4) Dynamic base speed
  int adjBase = (abs(error) > turnThresh) ? SLOW : FAST;

  // 5) Special pivot case (override)
  if (isOnlyLeftmostActive()) {
    ChangeWheelSpeeds(0,  -adjBase, 0,  +adjBase);  // spin left
  }
  else if (isOnlyRightmostActive()) {
    ChangeWheelSpeeds(0,  +adjBase, 0,  -adjBase);  // spin right
  }
  else {
    // normal proportional steering
    ChangeWheelSpeeds(currSpeedL,
                      adjBase + delta,
                      currSpeedR,
                      adjBase - delta);
    currSpeedL = adjBase + delta;
    currSpeedR = adjBase - delta;
  }
}
