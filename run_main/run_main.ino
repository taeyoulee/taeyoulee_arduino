#include <Motors.h>
#include "DualVNH5019MotorShield.h"
#include "SingleWheelEncoders.h"
#include "PID_v1.h"
#include "PinChangeInt.h"
#include <cstring.h>

const double P_FAST = 0.01;
const double I_FAST = 0.045;
double adjustment;

Motors motors;
SingleWheelEncoders swe;

const int NUM_SENSOR_READINGS = 5;

String content="";  //read serial input
char character;       //store the serial input one by one

void setup()
{
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  motors.init(13,11);
}

void loop() {
  
  while(Serial.available()) {
      character = Serial.read();
      if (character == '/'){
        break;
      }
      else if (character >= 48 && character <= 122){  //ascii 0-z
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
            break;
          case 'l':
          case 'L': //turn left
            if (content.charAt(3) == '1'){
             motors.turnLeftFast();
           }
           else{
              motors.turnLeft();
           }
            break;
          case 'r':
          case 'R': //turn right
            if (content.charAt(3) == '1'){
             motors.turnRightFast();
           }else{             
            motors.turnRight();
           }
            break;
          case 'b':
          case 'B':
            blocks = content.substring(1,3).toInt();
            motors.moveBackward(blocks);
            break;
          case 'a':
          case 'A':
            adjustCorner();
            break;
          default:             
            break;
         } 
         if (content.charAt(3) == '1'){
           Serial.println("SP"); 
         }
         else {
            readSensor();
         }        
         content = "";
         Serial.flush();
         //delay(100);
       }  
       
}

void readSensor(){
  String IRReading = "s";  
  //IRReading = IRReading + centerLeftMeasure();
  IRReading = IRReading + forwardLeftMeasure();
  IRReading = IRReading + leftForwardMeasure();
  IRReading = IRReading + forwardMeasure();
  IRReading = IRReading + rightForwardMeasure();
  IRReading = IRReading + rightMeasure();
  Serial.println(IRReading); 
  
}




int centerLeftMeasure (){ 
  double measures = 0.0;
  int count = NUM_SENSOR_READINGS;
  for (int i=0; i<NUM_SENSOR_READINGS; i++){
    float reading = analogRead(A1);
    if (reading > 328){
      measures += 1;
    }
    else if (reading > 209 && reading <= 328){
      measures += 2;
    }
    else {
      measures += 3;
    }
    delay(40);
  }
  int blocks = (int) (measures/count + 0.5);  //round to nearest integer
  if (blocks == 3){
    return 0; 
  }
  else {
    return blocks ;
  }
}

int forwardLeftMeasure (){ 
  double measures = 0.0;
  int count = NUM_SENSOR_READINGS;
  for (int i=0; i<NUM_SENSOR_READINGS; i++){
    float reading = analogRead(A3);
    if (reading > 503){
      measures += 1;
    }
    else if (reading > 390 && reading <= 503){
      measures += 2;
    }
    else if (reading > 310 && reading <= 390){
      measures += 3;
    }
    else if (reading > 257 && reading <= 310){
      measures += 4;
    }
    else {
      measures += 5;
    }
    delay(40);
  }
  int blocks = (int) (measures/count + 0.5);  //round to nearest integer
  if (blocks == 5){
    return 0; 
  }
  else {
    return blocks ;
  }
}



int leftForwardMeasure (){ 
  double measures = 0.0;
  int count = NUM_SENSOR_READINGS;
  for (int i=0; i<NUM_SENSOR_READINGS; i++){
    float reading = analogRead(A0);
    if (reading > 339){
      measures += 1;
    }
    else if (reading > 225 && reading <= 339){
      measures += 2;
    }
    else if (reading > 165 && reading <= 225){
      measures += 3;
    }
    else {
      measures += 4;
    }
    delay(40);
  }
  int blocks = (int) (measures/count + 0.5);  //round to nearest integer
  if (blocks == 4){
    return 0; 
  }
  else {
    return blocks ;
  }
}


int forwardMeasure(){ 
  double measures = 0.0;
  int count = NUM_SENSOR_READINGS;
  for (int i=0; i<NUM_SENSOR_READINGS; i++){
    float reading = analogRead(A2);
    if (reading > 400){
      measures += 1;
    }
    else if (reading > 231 && reading <= 400){
      measures += 2;
    }
    else if (reading > 168 && reading <= 231){
      measures += 3;
    }
    else {
      measures += 4;
    }
    delay(40);
  }
  int blocks = (int) (measures/count + 0.5);  //round to nearest integer
  if (blocks == 4){
    return 0; 
  }
  else {
    return blocks ;
  }
}

int rightForwardMeasure (){ 
  double measures = 0.0;
  int count = NUM_SENSOR_READINGS;
  for (int i=0; i<NUM_SENSOR_READINGS; i++){
    float reading = analogRead(A4);
    if (reading > 344){
      measures += 1;
    }
    else if (reading > 215 && reading <= 344){
      measures += 2;
    }
    else if (reading > 160 && reading <= 215){
      measures += 3;
    }
    else {
      measures += 4;
    }
    delay(40);
  }
  int blocks = (int) (measures/count + 0.5);  //round to nearest integer
  if (blocks == 4){
    return 0; 
  }
  else {
    return blocks ;
  }
}

int rightMeasure (){  
  double measures = 0.0;
  int count = NUM_SENSOR_READINGS;
  for (int i=0; i<NUM_SENSOR_READINGS; i++){
    float reading = analogRead(A5);
    if (reading > 502){
      measures += 1;
    }
    else if (reading > 380 && reading <= 502){
      measures += 2;
    }
    else if (reading > 299 && reading <= 380){
      measures += 3;
    }
    else if (reading > 248 && reading <= 299){
      measures += 4;
    }
    else if (reading > 210 && reading <= 248){
      measures += 5;
    }
    else {
      measures += 6;
    }
    delay(40);
  }
  int blocks = (int) (measures/count + 0.5);  //round to nearest integer
  if (blocks == 6){
    return 0; 
  }
  else {
    return blocks ;
  }
}


void adjustCorner(){
  //adjust angle
  int diffLR = -8;
  //int diffLR = 15;
  int leftReading = readLeftForwardDistance();
  int rightReading = readRightForwardDistance() + diffLR;
  int reading = abs( leftReading - rightReading);
  int angle;
  while (reading > 10){    
    leftReading = readLeftForwardDistance();
    rightReading = readRightForwardDistance() + diffLR;
    reading = abs( leftReading - rightReading);
    angle = reading/6;
    if (leftReading > rightReading){        
      motors.rotate(angle, true);  //rotate left
    }
    else {
      motors.rotate(angle, false);  //rotate right
    }
    //delay(40); 
     delay(5);   
  }
  
  //adjust distance- front
  while (leftReading<450 || leftReading>470){
    leftReading = analogRead(A0);
    int movement = abs(leftReading-460);
    if (leftReading>470 ){  //backwards
      motors.move(movement, true, true, 80.0, 80.0);
    }
    else {    //forward
      motors.move(movement, false, false, 80.0, 80.0);
    }
    delay(40);    
  }
  
  //adjust distance - left
  unsigned long left_measure = readLeftDistance();
  delay(100);
  motors.turnRight();  //turn right
  delay(100);
  //adjsut offset
  int adjust_offset = 420;
  if (left_measure < adjust_offset){
    int offset = (int)((adjust_offset - left_measure)*2.1);  //200 is 8cm    
    motors.move(offset, true, true, 80.0, 80.0);  //move backward
  }
  else if (left_measure > adjust_offset){
    int offset = (int)(((double)left_measure - adjust_offset)*1.4);
    motors.move(offset, false, false, 80.0, 80.0);  //move forward
  }
  
}

int readLeftForwardDistance (){ 
  double measures = 0.0;
  int count = NUM_SENSOR_READINGS;
  for (int i=0; i<NUM_SENSOR_READINGS; i++){
    measures += analogRead(A0);
    delay(40);
  }
  return (int) (measures/count);  //round to nearest integer
}
int readRightForwardDistance (){ 
  double measures = 0.0;
  int count = NUM_SENSOR_READINGS;
  for (int i=0; i<NUM_SENSOR_READINGS; i++){
    measures += analogRead(A4);
    delay(40);
  }
  return (int) (measures/count);  //round to nearest integer
}

int readLeftDistance (){ 
  double measures = 0.0;
  int count = NUM_SENSOR_READINGS;
  for (int i=0; i<NUM_SENSOR_READINGS; i++){
    measures += analogRead(A1);
    delay(40);
  }
  return (int) (measures/count);  //round to nearest integer
}





