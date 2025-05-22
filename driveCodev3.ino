
#include <ECE3.h>
#include <stdio.h>
// doesn't go backwards on the turns but does on the straight
const int left_nslp_pin=31; // nslp HIGH ==> awake & ready for PWM
const int right_nslp_pin=11; // nslp HIGH ==> awake & ready for PWM
const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

const int LED_RF = 75;
int baseSpeed = 22;
int currSpeedL = 0;
int currSpeedR = 0;

uint16_t sensorValues[8];
int weights[8] = {-4, -4, -2, -1, 1, 2, 4, 4};
int mins[8] = {804, 736, 716, 733, 641, 628, 733, 710};
int maxes[8] = {1696, 1324, 1387, 1185, 940, 914, 1767, 1790};

int errLen = 2;
int pastErrors[2] = {0, 0};
float kp = -0.015;
float kd = -0.03;
int iters = 0;

int stateR = 0;
int stateL = 0;

int getP(int error) {
  return error*kp;
}

int getD(int arr[]) {
  return kd*((arr[0] + arr[1]) / errLen);
}

int splitIters = 0;

// int detectSplit(uint16_t vals[]) { // Vishal 5/19 change for left code. S turn may no longer work. REMOVE?
//   // new idea, if middle 3 vals sum is above a certain value , we aren't in split
//   // 
//   if((vals[3]+vals[4]+vals[5])>5000){ // 5000 is a magic number
//     return 0; //no split
//   }
//   return 1; //otherwise split detected
// }

int detectCross(uint16_t vals[]) {
  if (vals[0] > 2300 && vals[1] > 2300 && vals[2] > 2200 && vals[3] > 1800 
  && vals[4] > 1800 && vals[5] > 2200 && vals[6] > 2300 && vals[7] > 2300) {
    return 1;
  }
  return 0;
}

int altdetectSplit(uint16_t vals[]) {
  if (detectCross(vals)) return 0;
  if (vals[2] + vals[3] > 2500) {
    return 1;
  }
  return 0;

}

int detectSplit(uint16_t vals[]){
  if (detectCross(vals)) return 0;
  int changes=0;
  int minBound = 1100;
  int maxBound = 1500;
  for(int i=0; i<7; i++){
    if(vals[i]<minBound && vals[i+1]>maxBound || vals[i]>maxBound && vals[i+1]<minBound){
      changes++;
    }
  }
  Serial.print("Changes: ");
  Serial.println(changes);
  if(changes >= 3){
    return 1;
  }
  return 0;
}

int computeError(uint16_t vals[]) {
  int sum = 0;

  for (int i = 0; i < 8; i++) {
    int norm = 0;
    if (vals[i] - mins[i] < 0) vals[i] = 0;
    else vals[i] -= mins[i];

    vals[i] = (vals[i]*1000)/maxes[i];

    sum += weights[i]*vals[i];
  }
  return sum / 4;
}

void  ChangeWheelSpeeds(int finalLeftSpd, int finalRightSpd) {
/*  
 *   This function changes the car speed gradually (in about 30 ms) from initial
 *   speed to final speed. This non-instantaneous speed change reduces the load 
 *   on the plastic geartrain, and reduces the failure rate of the motors. 
 */


    if (finalLeftSpd < 0) {
      digitalWrite(left_dir_pin,HIGH);
      analogWrite(left_pwm_pin,finalLeftSpd*-1);   
    } else {
      digitalWrite(left_dir_pin,LOW);
      analogWrite(left_pwm_pin,finalLeftSpd);   
    }

    if (finalRightSpd < 0) {
      digitalWrite(right_dir_pin,HIGH);
      analogWrite(right_pwm_pin,finalRightSpd*-1); 
    } else {
      digitalWrite(right_dir_pin,LOW);
      analogWrite(right_pwm_pin,finalRightSpd); 
    }


 
} // end void  ChangeWheelSpeeds
void setup() {

  ECE3_Init();

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

//  pinMode(13,INPUT);

  pinMode(LED_RF, OUTPUT);

  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH);
//  digitalWrite(left_nslp_pin,LOW);
//  digitalWrite(right_nslp_pin,LOW);
  
// set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 

  resetEncoderCount_left();
  resetEncoderCount_right();

  digitalWrite(left_dir_pin,LOW);  // Set car direction to forward
  digitalWrite(right_dir_pin,LOW);

  delay(2000); //Wait 2 seconds before starting 
}

int crossIters = 0;
int turnCount = 0;
int turnVal = 70;

void loop() {
  // put your main code here, to run repeatedly:
  ECE3_read_IR(sensorValues);

  for (unsigned char i = 0; i < 8; i ++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }

  if(detectCross(sensorValues)) {
    crossIters++;
    // ChangeWheelSpeeds(0, 0);
    // delay(1000);
    Serial.print("Cross detected");
    if (crossIters > 2) {
        ChangeWheelSpeeds(0, 0);
        delay(2000);
        ChangeWheelSpeeds(30, -30);
        turnVal *= -1;
        delay(2200);
        crossIters = 0;
    } else return;
  }

  

  // splitIters += detectSplit(sensorValues); //splitIters would never be reset? I think? 
  if(detectSplit(sensorValues)){
    splitIters++;

  }else{
    splitIters=0;
  }

  int splitTurn = 0;
  if (splitIters >= 5) { //if we've detected a split for 10 iterations in a row. Maybe change 10 to a smaller number like 5?
    // weights[0] = 0;
    // weights[1] = 0;
    // weights[2] = 0;
    // weights[3] = 0;
    // if (!detectSplit(sensorValues)) {
    //   splitIters = 0;
    //   weights[0] = -8;
    //   weights[1] = -4;
    //   weights[2] = -2;
    //   weights[3] = -1;
    // }
    //commented out code ^ is idea that caused car to just oscillate between left and right paths
    splitTurn = turnVal; // previous value: 10 (possibly increase)

  }
  Serial.println(detectSplit(sensorValues));

  int error = computeError(sensorValues);
  // Serial.println(error);
  // delay(500);

  pastErrors[iters] = error;
  iters = (iters + 1) % 2;

  int delta = getP(error) + getD(pastErrors);
  
  int speedL = baseSpeed + delta - splitTurn;
  int speedR = baseSpeed - delta + splitTurn;
  ChangeWheelSpeeds(speedL, speedR);
  // currSpeedL = baseSpeed + delta;
  // currSpeedR = baseSpeed - delta;
  // Serial.print("Right: ");
  // Serial.println(currSpeedL);

  // Serial.print("Left: ");
  // Serial.println(currSpeedR);

}