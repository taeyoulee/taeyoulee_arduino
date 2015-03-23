
#include <Motors.h>
#include "DualVNH5019MotorShield.h"
#include "SingleWheelEncoders.h"
#include "PID_v1.h"
#include "PinChangeInt.h"
#include <cstring.h>

const char LEFT_SENSOR = 2;//2
const char MID_SENSOR = 1;
const char RIGHT_SENSOR = 0;//0
const char LEFTSIDE_SENSOR = 3;
const char RIGHTSIDE_SENSOR = 4;//0

const double P_FAST = 0.01;
const double I_FAST = 0.045;
double adjustment;
boolean done=true;

Motors motors;
SingleWheelEncoders swe;

const int NUM_SENSOR_READINGS = 5;

String content="";  //read serial input
char character;       //store the serial input one by one


void setup() {
  Serial.begin(115200);
  //Serial.println("Dual VNH5019 Motor Shield");
  motors.init(11,13);
}  

void loop() {

  //Serial.print("Sensor left: ");
  int distLeft = readLeftIR()/10;
  if(distLeft>9)
  {
    distLeft=-1;
  }
  //Serial.println(distLeft);
  //delay(500);
  //Serial.print("Sensor mid: ");
  int distMid = readMidIR()/10;
  if(distMid>9)
  {
    distMid=-1;
  }
  //Serial.println(distMid);
  
  //Serial.print("Sensor right: ");
  int distRight = readRightIR()/10;
  //Serial.println(distRight);
  //Serial.println("--------");
  if(distRight>9)    
  {
    distRight=-1;
  }
  
  //Serial.print("Sensor rightSide: ");
  int distRightSide = readRightSideIR()/10;
  //Serial.println(distRightSide);
  //Serial.println("--------");
  if(distRightSide>9)
  {
    distRightSide=-1;
  }
  
  //Serial.print("Sensor leftSide: ");
  int distLeftSide = readLeftSideIR()/10;
  if(distLeftSide>9)
  {
    distLeftSide=-1;
  }

   if(done==true)
   {
     String sensorOutput="";
     sensorOutput.concat(distLeft);
     sensorOutput.concat(",");
     sensorOutput.concat(distMid);
     sensorOutput.concat(",");
     sensorOutput.concat(distRight);
     sensorOutput.concat(",");
     sensorOutput.concat(distLeftSide);
     sensorOutput.concat(",");
     sensorOutput.concat(distRightSide);
     sensorOutput.concat(",");
     Serial.println(sensorOutput);
    Serial.flush();
    done=false;
    //delay(1000);
   }
   else
   {
     delay(10);
   }
  while(Serial.available()) {
      character = Serial.read();
      if (character == '/'){
        break;
      }
      else if (character==46 || (character >= 48 && character <= 122)){  //ascii 0-z
        content.concat(character);
      }
      delay(30);
  }

  if (content.length() >=4) {
    int blocks;
    switch (content[0]) {
      case 'f':
      case 'F': //forward
        blocks = content.substring(1,3).toInt();
        motors.moveForward(blocks);
        done=true;
        break;
      case 'l':
      case 'L': //turn left
        if (content.charAt(3) == '1'){
         motors.turnLeftFast();
         done=true;
       }
       else{
          motors.turnLeft();
          done=true;
       }
        break;
      case 'r':
      case 'R': //turn right
        if (content.charAt(3) == '1'){
         motors.turnRightFast();
         
         done=true;
       }else{             
        motors.turnRight();
        done=true;
       }
        break;
      case 'b':
      case 'B':
        blocks = content.substring(1,3).toInt();
        motors.moveBackward(blocks);
        done=true;
        break;
      case 'W':
      case 'w':
        motors.calibrate(content[1],content.substring(2).toFloat());
        break;
      case 'c':
      case 'C':
       if (content.charAt(2) == '0'){
        adjustLeftCorner();
       }else{
        adjustRightCorner();
       }
       break;
      case 'a':
      case 'A':
        if (content.charAt(2) == '0'){
          adjustLeftWall();
         }else{
          adjustRightWall();
         }
       break;
      default:             
        break;
     } 
     if (content.charAt(3) == '1'){
       //Serial.println("SP"); 
     }
     // else {
     //    //readSensor();
     // }        
     content = "";
     Serial.flush();
     //delay(10);
     //delay(100);
   }else{
     Serial.flush();
   }  
}

////////////////////////////////////Aaron's sensor////////////////////////////////////
//Method to read various sensor, each read is about 39ms
//which will take 390ms, will investigate further to lower this delay
float readLeftIR() {
  int adcValue = readSensor(LEFT_SENSOR);
  //error condition or undetectable range
  if(adcValue == -1)
    return -1; 
  return (pow(3027.4 / adcValue, 1.2134)); 
}

int readMidIR() {
  int adcValue = readSensor(MID_SENSOR);
  //error condition or undetectable range
  if(adcValue == -1)
    return -1;  
  return (pow(3027.4 / adcValue, 1.2134));  

}

int readRightSideIR() {
  int adcValue = readSensor(RIGHTSIDE_SENSOR);
  //error condition or undetectable range
  if(adcValue == -1)
    return -1; 
  return (pow(3027.4 / adcValue, 1.2134));  
}

int readLeftSideIR() {
  int adcValue = readSensor(LEFTSIDE_SENSOR);
  //error condition or undetectable range
  if(adcValue == -1)
    return -1;
  return (pow(3027.4 / adcValue, 1.2134));    
}

int readRightIR() {
  int adcValue = readSensor(RIGHT_SENSOR);
  //error condition or undetectable range
  if(adcValue == -1)
    return -1;
  return (pow(3027.4 / adcValue, 1.2134));  
}

int readSensor(char sensorPin) {
  char i;
  long sensorReadings = 0;
  int adcValues[10];
  for(i=0; i<10; i++) {
    sensorReadings = analogRead(sensorPin);
    adcValues[i] = sensorReadings;//analogRead(sensorPin);
    delay(5);
  }
  
  return filterNoise(adcValues); 
}

int filterNoise(int adcValues[10]) { 
    //apply bubblesort and then take the median value 
    int c, d, swap, _average = 4, _threshold = 3, i, _size = 10; 
    int sum = 0;
    for (c = 0 ; c < (_size - 1); c++) { 
        for (d = 0 ; d < _size - c - 1; d++) { 
            if (adcValues[d] > adcValues[d+1]) { 
                swap = adcValues[d]; 
                adcValues[d] = adcValues[d+1]; 
                adcValues[d+1] = swap; 
            } 
        } 
    } 
    for(i=_threshold; i<_size-_threshold; i++) {
      sum += adcValues[i];
      //Serial.println(adcValues[i]);
    }
    return sum/_average;
} 

////////////////////////////////////Self-calibration function////////////////////////////////////
//void adjustLeftWall(){
//  int angle;
//  int distLeftCM;
//  int distRightCM;
//  int i;
//  //motors.turnLeftAdjust();
//  //read the left and mid sensors
//  distLeftCM=readLeftForwardDistance();
//  distRightCM=readMidDistance();
//  if (distLeftCM<1000 && distRightCM<1000){
//
//    int readingDiff=abs(distLeftCM-distRightCM);
//    //while there is an angle difference
//    //while (readingDiff > 10){
//    for (i=0; i<10; i++){
//      distLeftCM=readLeftForwardDistance();
//      distRightCM=readMidDistance()+10;
//      Serial.println("The left sensor");
//      Serial.println(distLeftCM);
//      Serial.println("The mid sensor");
//      Serial.println(distRightCM);
//      readingDiff=abs(distLeftCM-distRightCM);
//      angle = readingDiff/24;
//      if (distLeftCM > distRightCM){        
//          motors.rotateAdjust(angle, true);  //rotate left
//        }
//        else {
//          motors.rotateAdjust(angle, false);  //rotate right
//        }
//        delay(50);
//    }
//
//    //while there is  a displacement
//    while(distLeftCM>=605 ||distLeftCM<=595){
//      distLeftCM=readLeftForwardDistance();
//      int movement= abs(distLeftCM-600);
//      if(distLeftCM > 600){//backwards
//        motors.moveAdjust(movement, true, true, 80.0, 80.0);
//      }else{
//        motors.moveAdjust(movement, false, false, 80.0, 80.0);
//      }
//    }
//    //motors.turnRightAdjust();
//  }else{
//    //motors.turnRightAdjust();
//  }
//  delay(50);
//}


void adjustLeftWall(){
  int angle;
  int distLeftCM;
  int distRightCM;
  int i;
  motors.turnLeftAdjust();
  //read the left and mid sensors
  distLeftCM=readSensor(2);
  distRightCM=readSensor(0);
  if (distLeftCM<1000 && distRightCM<1000){
    int readingDiff=abs(distLeftCM-distRightCM);
    //while there is an angle difference
    while (readingDiff > 10){
    //for (i=0; i<10; i++){
      distLeftCM=readSensor(2);
      distRightCM=readSensor(0);
//      Serial.println("The left sensor");
//      Serial.println(distLeftCM);
//      Serial.println("The mid sensor");
//      Serial.println(distRightCM);
      readingDiff=abs(distLeftCM-distRightCM);
      angle = readingDiff/20;
      if (distLeftCM > distRightCM){        
          motors.rotateAdjust(angle, true);  //rotate left
        }
        else {
          motors.rotateAdjust(angle, false);  //rotate right
        }
        delay(50);
    }

    //while there is  a displacement
    while(distLeftCM>=605 ||distLeftCM<=595){
      distLeftCM=readLeftForwardDistance();
      int movement= abs(distLeftCM-600);
      if(distLeftCM > 600){//backwards
        motors.moveAdjust(movement, true, true, 80.0, 80.0);
      }else{
        motors.moveAdjust(movement, false, false, 80.0, 80.0);
      }
    }
    motors.turnRightAdjust();
  }else{
    motors.turnRightAdjust();
  }
  delay(50);
}

void adjustRightWall(){
  int angle;
  int distLeftCM;
  int distRightCM;

  motors.turnRightAdjust();
  //read the right and mid sensors
  distLeftCM=readSensor(2);
  distRightCM=readSensor(0);
  if (distLeftCM<1000 && distRightCM<1000){
    int readingDiff=abs(distLeftCM-distRightCM);
    //while there is an angle difference
    while (readingDiff > 10){
      distLeftCM=readSensor(2);
      distRightCM=readSensor(0);
      readingDiff=abs(distLeftCM-distRightCM);
      angle = readingDiff/24;
      if (distLeftCM > distRightCM){        
          motors.rotateAdjust(angle, true);  //rotate left
        }
        else {
          motors.rotateAdjust(angle, false);  //rotate right
        }
        delay(50);
    }

    //while there is  a displacement
    while(distLeftCM>=605 ||distLeftCM<=595){
      distLeftCM=readMidDistance();
      int movement= abs(distLeftCM-600);
      if(distLeftCM > 600){//backwards
        motors.moveAdjust(movement, true, true, 80.0, 80.0);
      }else{
        motors.moveAdjust(movement, false, false, 80.0, 80.0);
      }
    }
    motors.turnLeftAdjust();
  }else{
    motors.turnLeftAdjust();
  }
  delay(50);
}

void adjustLeftCorner(){
  int angle;
  int distLeftCM;
  int distRightCM;
  
  distLeftCM=readLeftForwardDistance();
  distRightCM=readRightForwardDistance();
  
  //if the sensor is too near to the wall
  if (distLeftCM < 485 || distRightCM < 485){
    while(distLeftCM>=605 ||distLeftCM<=595){
      distLeftCM=readLeftForwardDistance();
      int movement= abs(distLeftCM-600);
      if(distLeftCM > 600){//backwards
        motors.moveAdjust(movement, true, true, 80.0, 80.0);
      }else{
        motors.moveAdjust(movement, false, false, 80.0, 80.0);
      }
      delay(50);
    }
  }

  //if there is a wall
  if (distLeftCM<1000 && distRightCM<1000){
    int readingDiff=abs(distLeftCM-distRightCM);
    //while there is still an angle difference
    while (readingDiff > 10){
      distLeftCM=readLeftForwardDistance();
      distRightCM=readRightForwardDistance();
      readingDiff=abs(distLeftCM-distRightCM);
      angle = readingDiff/24;
      if (distLeftCM > distRightCM){        
        motors.rotateAdjust(angle, true);  //rotate left
      }
      else {
        motors.rotateAdjust(angle, false);  //rotate right
      }
      delay(50);
    }
    //while there is  a displacement
    while(distLeftCM>=605 ||distLeftCM<=595){
      distLeftCM=readLeftForwardDistance();
      int movement= abs(distLeftCM-600);
      if(distLeftCM > 600){//backwards
        motors.moveAdjust(movement, true, true, 80.0, 80.0);
      }else{
        motors.moveAdjust(movement, false, false, 80.0, 80.0);
      }
      delay(50);
    }
  }

  motors.turnLeftAdjust();
  distLeftCM=readLeftForwardDistance();
  distRightCM=readRightForwardDistance();
  //if there is a wall
  if (distLeftCM<1000 && distRightCM<1000){
    distLeftCM=readLeftForwardDistance();
    distRightCM=readRightForwardDistance();
    int readingDiff=abs(distLeftCM-distRightCM);
    //while there is still an angle difference
    while (readingDiff > 10){
      distLeftCM=readLeftForwardDistance();
      distRightCM=readRightForwardDistance();
      readingDiff=abs(distLeftCM-distRightCM);
      angle = readingDiff/24;
      if (distLeftCM > distRightCM){        
        motors.rotateAdjust(angle, true);  //rotate left
      }
      else {
        motors.rotateAdjust(angle, false);  //rotate right
      }
      delay(50);
    }
    //while there is  a displacement
    while(distLeftCM>=605 ||distLeftCM<=595){
      distLeftCM=readLeftForwardDistance();
      int movement= abs(distLeftCM-600);
      if(distLeftCM > 600){//backwards
        motors.moveAdjust(movement, true, true, 80.0, 80.0);
      }else{
        motors.moveAdjust(movement, false, false, 80.0, 80.0);
      }
    }
    motors.turnRightAdjust();
  }else{
    motors.turnRightAdjust();
  }
  delay(50);
}

void adjustRightCorner(){
  int angle;
  int distLeftCM;
  int distRightCM;
  
  distLeftCM=readLeftForwardDistance();
  distRightCM=readRightForwardDistance();
  //if there is a wall
  if (distLeftCM<1000 && distRightCM<1000){
    int readingDiff=abs(distLeftCM-distRightCM);
    //while there is still an angle difference
    while (readingDiff > 10){
      distLeftCM=readLeftForwardDistance();
      distRightCM=readRightForwardDistance();
      readingDiff=abs(distLeftCM-distRightCM);
      angle = readingDiff/24;
      if (distLeftCM > distRightCM){        
        motors.rotateAdjust(angle, true);  //rotate left
      }
      else {
        motors.rotateAdjust(angle, false);  //rotate right
      }
      delay(50);
    }
    //while there is  a displacement
    while(distLeftCM>=605 ||distLeftCM<=595){
      distLeftCM=readLeftForwardDistance();
      int movement= abs(distLeftCM-600);
      if(distLeftCM > 600){//backwards
        motors.moveAdjust(movement, true, true, 80.0, 80.0);
      }else{
        motors.moveAdjust(movement, false, false, 80.0, 80.0);
      }
      delay(50);
    }
  }

  motors.turnRightAdjust();
  distLeftCM=readLeftForwardDistance();
  distRightCM=readRightForwardDistance();
  //if there is a wall
  if (distLeftCM<1000 && distRightCM<1000){
    distLeftCM=readLeftForwardDistance();
    distRightCM=readRightForwardDistance();
    int readingDiff=abs(distLeftCM-distRightCM);
    //while there is still an angle difference
    while (readingDiff > 10){
      distLeftCM=readLeftForwardDistance();
      distRightCM=readRightForwardDistance();
      readingDiff=abs(distLeftCM-distRightCM);
      angle = readingDiff/24;
      if (distLeftCM > distRightCM){        
        motors.rotateAdjust(angle, true);  //rotate left
      }
      else {
        motors.rotateAdjust(angle, false);  //rotate right
      }
      delay(50);
    }
    //while there is  a displacement
    while(distLeftCM>=605 ||distLeftCM<=595){
      distLeftCM=readLeftForwardDistance();
      int movement= abs(distLeftCM-600);
      if(distLeftCM > 600){//backwards
        motors.moveAdjust(movement, true, true, 80.0, 80.0);
      }else{
        motors.moveAdjust(movement, false, false, 80.0, 80.0);
      }
    }
    motors.turnLeftAdjust();
  }else{
    motors.turnLeftAdjust();
  }
  delay(50);
}

///////////////////////////////Liu Xiao's Sensor reading/////////////////////////////////
int readLeftForwardDistance (){ 
  double measures = 0.0;
  int count = NUM_SENSOR_READINGS;
  for (int i=0; i<NUM_SENSOR_READINGS; i++){
    measures += analogRead(A2);
    delay(5);
  }
  return (int) (measures/count);  //round to nearest integer
}
int readMidDistance (){ 
  double measures = 0.0;
  int count = NUM_SENSOR_READINGS;
  for (int i=0; i<NUM_SENSOR_READINGS; i++){
    measures += analogRead(A1);
    delay(5);
  }
  return (int) (measures/count);  //round to nearest integer
}
int readRightForwardDistance (){ 
  double measures = 0.0;
  int count = NUM_SENSOR_READINGS;
  for (int i=0; i<NUM_SENSOR_READINGS; i++){
    measures += analogRead(A0);
    delay(5);
  }
  return (int) (measures/count);  //round to nearest integer
}
int readLeftDistance (){ 
  double measures = 0.0;
  int count = NUM_SENSOR_READINGS;
  for (int i=0; i<NUM_SENSOR_READINGS; i++){
    measures += analogRead(A3);
    delay(5);
  }
  return (int) (measures/count);  //round to nearest integer
}
int readRightDistance (){ 
  double measures = 0.0;
  int count = NUM_SENSOR_READINGS;
  for (int i=0; i<NUM_SENSOR_READINGS; i++){
    measures += analogRead(A4);
    delay(5);
  }
  return (int) (measures/count);  //round to nearest integer
}

