
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
       adjustCorner();
       break;
      case 'a':
      case 'A':
       adjustLeftWall();
       break;
      default:             
        break;
     } 
     if (content.charAt(3) == '1'){
       Serial.println("SP"); 
     }
     else {
        //readSensor();
     }        
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
void adjustLeftWall(){
  int angle;
  int distLeftCM;
  int distRightCM;

  int distLeftSideCM=readLeftDistance();
//  Serial.println("distleftSideCM");
//  Serial.println(distLeftSideCM);
  //if there is displacement
  //if ( (distLeftSideCM<=1000 && distLeftSideCM >= 800) ||distLeftSideCM <= 620){
    motors.turnLeftAdjust();
    distLeftCM=readLeftForwardDistance();
    distRightCM=readRightForwardDistance();
//    Serial.println("is there 3 obstacles?");
//    Serial.println(distLeftCM);
//    Serial.println(distRightCM);
    //if there is a wall
    if (distLeftCM<1000 && distRightCM<1000){
      distLeftCM=readLeftForwardDistance();
      distRightCM=readRightForwardDistance();
      int readingDiff=abs(distLeftCM-distRightCM);
//      Serial.println("entering the adjustleftwall");
//      Serial.println(distLeftCM);
//      Serial.println(distRightCM);
      //while there is still an angle difference
      while (readingDiff > 10){
//        Serial.println("entering the first while");
        distLeftCM=readLeftForwardDistance();
        distRightCM=readRightForwardDistance() -8;
        readingDiff=abs(distLeftCM-distRightCM);
        angle = readingDiff/12;
//        Serial.print("Angle");
//        Serial.println(angle);
        if (distLeftCM > distRightCM){        
          motors.rotateAdjust(angle, true);  //rotate left
        }
        else {
          motors.rotateAdjust(angle, false);  //rotate right
        }
        delay(5);
      }
//      Serial.println(distLeftCM);
//      Serial.println(distRightCM);
      //while there is  a displacement
      while(distLeftCM>=605 ||distLeftCM<=595){
//        Serial.println("entering the second while");
//        Serial.println(distLeftCM);
        distLeftCM=readLeftForwardDistance();
        int movement= abs(distLeftCM-600);
        if(distLeftCM > 600){//backwards
          motors.moveAdjust(movement, true, true, 80.0, 80.0);
        }else{
          motors.moveAdjust(movement, false, false, 80.0, 80.0);
        }
      }
      //Serial.println("turn right");
      motors.turnRightAdjust();
    }else{
      motors.turnRightAdjust();
    }
  //}
}

void adjustCorner(){
  int angle;
  int distLeftCM;
  int distRightCM;
  
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
      distRightCM=readRightForwardDistance() -8;
      readingDiff=abs(distLeftCM-distRightCM);
      angle = readingDiff/12;
      if (distLeftCM > distRightCM){        
        motors.rotateAdjust(angle, true);  //rotate left
      }
      else {
        motors.rotateAdjust(angle, false);  //rotate right
      }
      delay(5);
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
  }

  int distLeftSideCM=readLeftDistance();
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
      distRightCM=readRightForwardDistance() -8;
      readingDiff=abs(distLeftCM-distRightCM);
      angle = readingDiff/12;
      if (distLeftCM > distRightCM){        
        motors.rotateAdjust(angle, true);  //rotate left
      }
      else {
        motors.rotateAdjust(angle, false);  //rotate right
      }
      delay(5);
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
