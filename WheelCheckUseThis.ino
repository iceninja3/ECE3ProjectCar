
// Motor Check code.
/*
 * This sketch tests for proper operation of the project car's motors.
 * The nominal motor PWM is 80, but the wheelSpd variable can be 
 * changed to any other value from 0 to 255.
 * 
 * The car moves forward for 800 encoder counts, then back for 800 encoder counts. It
 * repeats this oscillatory motion until the car is removed from the track.
 * 
 * The car startup and stop functions are gradually changed by the 
 * ChangeWheelSpeeds() function. This is meant to reduce the stress on the plastic
 * gear sets in the motors, prolonging motor life.
 */
 
#include <ECE3.h>
#include <stdio.h>

const int left_nslp_pin=31; // nslp HIGH ==> awake & ready for PWM
const int right_nslp_pin=11; // nslp HIGH ==> awake & ready for PWM
const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

const int LED_RF = 75;
int wheelSpd = 80;
int distance = 300;


///////////////////////////////////
void setup() {
// put your setup code here, to run once:

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

  delay(2000); //Wait 2 seconds before starting 
  
}

void loop() {

  digitalWrite(left_dir_pin,LOW);  // Set car direction to forward
  digitalWrite(right_dir_pin,LOW);
  
  ChangeWheelSpeeds(0, wheelSpd, 0, wheelSpd); // Stat car up
/*  analogWrite(left_pwm_pin,wheelSpd); // Maintain speed 
  analogWrite(right_pwm_pin,wheelSpd); */
  while(average() < distance); // Maintain speed 
  ChangeWheelSpeeds(wheelSpd, 0, wheelSpd, 0); //Stop car

//  Change direction to go backwards
  digitalWrite(left_dir_pin,HIGH); // Set car direction to  backward
  digitalWrite(right_dir_pin,HIGH);
  
  ChangeWheelSpeeds(0, wheelSpd, 0, wheelSpd); // Stat car up 
/*  analogWrite(left_pwm_pin,wheelSpd); // Maintain speed 
  analogWrite(right_pwm_pin,wheelSpd); */
  resetEncoderCount_left();
  resetEncoderCount_right();  
  while(average() < distance) ;
  ChangeWheelSpeeds(wheelSpd, 0, wheelSpd, 0); //Stop car
  
  resetEncoderCount_left();
  resetEncoderCount_right();  
 
} // end void loop()

//---------------------------------------------------------
void  ChangeWheelSpeeds(int initialLeftSpd, int finalLeftSpd, int initialRightSpd, int finalRightSpd) {
/*  
 *   This function changes the car speed gradually (in about 30 ms) from initial
 *   speed to final speed. This non-instantaneous speed change reduces the load 
 *   on the plastic geartrain, and reduces the failure rate of the motors. 
 */
  int diffLeft  = finalLeftSpd-initialLeftSpd;
  int diffRight = finalRightSpd-initialRightSpd;
  int stepIncrement = 20;
  int numStepsLeft  = abs(diffLeft)/stepIncrement;
  int numStepsRight = abs(diffRight)/stepIncrement;
  int numSteps = max(numStepsLeft,numStepsRight);
  
  int pwmLeftVal = initialLeftSpd;        // initialize left wheel speed 
  int pwmRightVal = initialRightSpd;      // initialize right wheel speed 
  int deltaLeft = (diffLeft)/numSteps;    // left in(de)crement
  int deltaRight = (diffRight)/numSteps;  // right in(de)crement

  for(int k=0;k<numSteps;k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(left_pwm_pin,pwmLeftVal);    
    analogWrite(right_pwm_pin,pwmRightVal); 
    delay(30);   
  } // end for int k
//  if(finalLeftSpd  == 0) analogWrite(left_pwm_pin,0); ;
//  if(finalRightSpd == 0) analogWrite(right_pwm_pin,0);
  analogWrite(left_pwm_pin,finalLeftSpd);  
  analogWrite(right_pwm_pin,finalRightSpd);  
} // end void  ChangeWheelSpeeds

//void ChangeBaseSpeed(int initialBaseSpd, int finalBaseSpd) {
//  /*  
// *   This function changes the car base speed gradually (in about 300 ms) from
// *   initialBaseSpeed to finalBaseSpeed. This non-instantaneous speed change
// *   reduces the load on the plastic geartrain, and reduces the failure rate of 
// *   the motors. 
// */
//  int speedChangeTime = 300; // milliseconds
//  int numSteps = 5;
//  int pwmVal = initialBaseSpd; // initialize left wheel speed 
//  int deltaSpeed = (finalBaseSpd-initialBaseSpd)/numSteps; // in(de)crement
//  for(int k=0;k<numSteps;k++) {
////    pwmVal = pwmLeftVal + deltaSpeed;
////    analogWrite(left_pwm_pin,pwmVal);    
////    analogWrite(right_pwm_pin,pwmVal); 
////    delay(60);   
//  } // end for int k
//} // end void ChangeBaseSpeed





int average()  //average pulse count
{
  int getL=getEncoderCount_left();
  int getR=getEncoderCount_right();
//  Serial.print(getL);Serial.print("\t");Serial.println(getR);
  return ((getEncoderCount_left() + getEncoderCount_right())/2);
}
