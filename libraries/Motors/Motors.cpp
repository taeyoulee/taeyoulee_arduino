#include "Motors.h"
#include <PID_v1.h>
#include <math.h>
#include <cstring.h>
 
   
// //ticks that required for one revolution of wheel, when moving at one block only
// int singleRevolutionCount = 1030; 
// //ticks for one revolution during continuous
// int ContSingleRevolutionCount = 1100; 

// double robotDiameter = 18.3;
// int wheelDiameter = 6;
// //tick count for moving one block
// int move10cmCount = (10.0 / (PI*wheelDiameter)) * singleRevolutionCount;  
// //tick count for moving one block during continuous movement
// int ContMove10cmCount = (10.0 / (PI*wheelDiameter)) * ContSingleRevolutionCount;  

// //calculate PID at every 10ms
// int pidLoopTime = 10; 
// //rotation per minute at normal speed mode
// double leftRPM = 150.0;  
// //right wheels seem to need faster RPM to maintain straight line movement
// double rightRPM = 149.9; 

// //rotation per minute at fast movement mode
// double leftRPMFast = 121.0; 
// double rightRPMFast = 120.0;
// String debug = "";

  int pidLoopTime = 10; 

Motors::Motors() {

}
 
void Motors::init(unsigned char m1a, unsigned char m2a)
{
  md.init();
  swe.init(m1a, m2a);//m1 is left , m2 is right

  robotDiameter = 18.3;
  wheelDiameter = 6;

  //ticks that required for one revolution of wheel, when moving at one block only
  singleRevolutionCount = 1035;
  //ticks for one revolution during continuous
  ContSingleRevolutionCount = 1110; 
  //rotation per minute at normal speed mode
  leftRPM = 200.0;  
  //right wheels seem to need faster RPM to maintain straight line movement
  rightRPM = 200.0; 
  //rotation per minute at fast movement mode
  leftRPMFast = 300.0; 
  rightRPMFast = 300.0;

  //tick count for moving one block
  move10cmCount = (10.0 / (PI*wheelDiameter)) * singleRevolutionCount;    
  //tick count for moving one block during continuous movement
  ContMove10cmCount = (10.0 / (PI*wheelDiameter)) * ContSingleRevolutionCount; 
  
  leftCounts=835;
  rightCounts=830;
  leftFastCounts=805;
  rightFastCounts=805;
}
 
void Motors::turnRight()
{
    move(rightCounts, false, true, leftRPM, rightRPM);
    delay(50);
}
void Motors::turnRightAdjust()
{
    moveAdjust(rightCounts, false, true, leftRPM, rightRPM);
    delay(100);
}
void Motors::turnRightFast()
{
    move(rightFastCounts, false, true, leftRPMFast, rightRPMFast);
    delay(200);
}
 
void Motors::turnLeft()
{
    move(leftCounts, true, false, leftRPM, rightRPM);
    delay(50);
}

void Motors::turnLeftAdjust()
{
    moveAdjust(leftCounts, true, false, leftRPM, rightRPM);
    delay(100);
}
 
void Motors::turnLeftFast()
{
    move(leftFastCounts, true, false, leftRPMFast, rightRPMFast);
    delay(200);
}
 
void Motors::moveForward(int blocks)
{
  if (blocks>1){
    move(ContMove10cmCount*blocks, false, false, leftRPMFast, rightRPMFast);
    delay(100);
  }
  else {
    move(move10cmCount*blocks, false, false, leftRPM, rightRPM);
  }
}
 
void Motors::moveBackward(int blocks)
{
  moveAdjust(move10cmCount*blocks, true, true , leftRPM, rightRPM);
}
 
void Motors::rotate(unsigned int degrees, bool direction)
{
  move(degrees*9.5, direction, (!direction), leftRPM, rightRPM);
}

void Motors::rotateAdjust(unsigned int degrees, bool direction)
{
  moveAdjust(degrees*9.5, direction, (!direction), leftRPM, rightRPM);
}

void Motors::calibrate(int whatToChange, double changeTo)
{
  switch(whatToChange){
    //walking straight
    case '1'://change leftRPM
      leftRPM=changeTo;
      //Serial.println(leftRPM);
      break;
    case '2':
      rightRPM=changeTo;
      //Serial.println(rightRPM);
      break;
    case '3':
      leftRPMFast=changeTo;
      //Serial.println(leftRPMFast);
      break;
    case '4':
      rightRPMFast=changeTo;
      //Serial.println(rightRPMFast);
      break;
    
    //turning
    case '5':
      leftCounts=changeTo;
      //Serial.println(leftCounts);
      break;
    case '6':
      rightCounts=changeTo;
      //Serial.println(rightCounts);
      break;
    case '7':
      leftFastCounts=changeTo;
      //Serial.println(leftFastCounts);
      break;
    case '8':
      rightFastCounts=changeTo;
      //Serial.println(rightFastCounts);
      break;
    //revolution counts
    case '9':
      singleRevolutionCount=changeTo;
      //Serial.println(singleRevolutionCount);
      move10cmCount = (10.0 / (PI*wheelDiameter)) * singleRevolutionCount;    
  
      break;
    case '0':
      ContSingleRevolutionCount=changeTo;
      //Serial.println(ContSingleRevolutionCount);
      ContMove10cmCount = (10.0 / (PI*wheelDiameter)) * ContSingleRevolutionCount; 
      break;

    default:
      break;
  }
}

void Motors::move(unsigned int counts, bool left, bool right, double leftRPM, double rightRPM)
{
  int leftMotorRunning = 1, rightMotorRunning = 1;
  // Wheel adjustment to move forward. To reverse set both left and right to true.
  int directionM1 = -1, directionM2 = 1; 
  unsigned long nowTime = 0, lastTime = 0;
  unsigned int m1Counts = 0, m2Counts = 0;
  unsigned int m1LastCounts = 0, m2LastCounts = 0;
  // setpoints are targeted rotations per minute.
  double inputM1 = 0.0, outputM1 = 0.0, setpointM1 = leftRPM; 
  double inputM2 = 0.0, outputM2 = 0.0, setpointM2 = rightRPM; 
 
  PID m1PID(&inputM1, &outputM1, &setpointM1, 0.3, 7.8, 0, DIRECT);
  PID m2PID(&inputM2, &outputM2, &setpointM2, 0.3, 7.8, 0, DIRECT);
  m1PID.SetMode(AUTOMATIC);
  m2PID.SetMode(AUTOMATIC);
  m1PID.SetSampleTime(pidLoopTime);
  m2PID.SetSampleTime(pidLoopTime);
  m1PID.SetOutputLimits(0, 400);
  m2PID.SetOutputLimits(0, 400);
   
  if (left) directionM1 = 1;
  if (right) directionM2 = -1;
  swe.getCountsAndResetM1();
  swe.getCountsAndResetM2();
  lastTime = millis();
  
  while (leftMotorRunning || rightMotorRunning) {   
    m1Counts = swe.getCountsM1();
    //Serial.print("M1Counts");
    //Serial.println(m1Counts, DEC);
    m2Counts = swe.getCountsM2();
    //Serial.print("M2Counts");
    //Serial.println(m2Counts, DEC);
    if (m1Counts >= counts) {
      leftMotorRunning=0;   
    }
    if (m2Counts >= counts) {
      rightMotorRunning=0; 
    }
    nowTime = millis();
    if (nowTime-lastTime >= pidLoopTime) { //Wait till pidLoopTime           
      lastTime = nowTime;
      inputM1 = ((double)(m1Counts - m1LastCounts) * 60 * 1000 / pidLoopTime)/(double)singleRevolutionCount;
      inputM2 = ((double)(m2Counts - m2LastCounts) * 60 * 1000 / pidLoopTime)/(double)singleRevolutionCount;
      m1PID.Compute();
      m2PID.Compute();
      md.setM1Speed((int)outputM1*directionM1);
      md.setM2Speed((int)outputM2*directionM2);
      m1LastCounts = m1Counts;
      m2LastCounts = m2Counts;    
    }
  }


    Serial.println("OK");

     
  md.setM1Brake(400);
  md.setM2Brake(400);
 
  swe.getCountsAndResetM1();
  swe.getCountsAndResetM2();
}

void Motors::moveAdjust(unsigned int counts, bool left, bool right, double leftRPM, double rightRPM)
{
  int leftMotorRunning = 1, rightMotorRunning = 1;
  // Wheel adjustment to move forward. To reverse set both left and right to true.
  int directionM1 = -1, directionM2 = 1; 
  unsigned long nowTime = 0, lastTime = 0;
  unsigned int m1Counts = 0, m2Counts = 0;
  unsigned int m1LastCounts = 0, m2LastCounts = 0;
  // setpoints are targeted rotations per minute.
  double inputM1 = 0.0, outputM1 = 0.0, setpointM1 = leftRPM; 
  double inputM2 = 0.0, outputM2 = 0.0, setpointM2 = rightRPM; 
 
 //Create PID instances
  PID m1PID(&inputM1, &outputM1, &setpointM1, 0.3, 7.8, 0, DIRECT);
  PID m2PID(&inputM2, &outputM2, &setpointM2, 0.3, 7.8, 0, DIRECT);
  m1PID.SetMode(AUTOMATIC);
  m2PID.SetMode(AUTOMATIC);
  m1PID.SetSampleTime(pidLoopTime);
  m2PID.SetSampleTime(pidLoopTime);
  m1PID.SetOutputLimits(0, 400);
  m2PID.SetOutputLimits(0, 400);
//Get initial control output
  if (left) directionM1 = 1;
  if (right) directionM2 = -1;
  swe.getCountsAndResetM1();
  swe.getCountsAndResetM2();
  lastTime = millis();
  
  //While loop to reduce the errors until they reach zero
  while (leftMotorRunning || rightMotorRunning) {   
    m1Counts = swe.getCountsM1();
    m2Counts = swe.getCountsM2();

    //break the while loop if the counts have reach setpoint
    if (m1Counts >= counts) {
      leftMotorRunning=0;   
    }
    if (m2Counts >= counts) {
      rightMotorRunning=0; 
    }
    nowTime = millis();
    if (nowTime-lastTime >= pidLoopTime) { //Wait till pidLoopTime           
      lastTime = nowTime;
      //convert the control output into RPM and input it to calculate PID
      inputM1 = ((double)(m1Counts - m1LastCounts) * 60 * 1000 / pidLoopTime)/(double)singleRevolutionCount;
      inputM2 = ((double)(m2Counts - m2LastCounts) * 60 * 1000 / pidLoopTime)/(double)singleRevolutionCount;
      m1PID.Compute();
      m2PID.Compute();
      //use control input to setspeed
      md.setM1Speed((int)outputM1*directionM1);
      md.setM2Speed((int)outputM2*directionM2);
      m1LastCounts = m1Counts;
      m2LastCounts = m2Counts;    
    }
  }

     
  md.setM1Brake(400);
  md.setM2Brake(400);
 
  swe.getCountsAndResetM1();
  swe.getCountsAndResetM2();
}
