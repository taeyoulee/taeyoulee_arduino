#include <Motors.h>
#include "DualVNH5019MotorShield.h"
#include "SingleWheelEncoders.h"
#include "PID_v1.h"
#include "PinChangeInt.h"
#include <cstring.h>
#include <Servo.h>                                  // Include Servo library

Servo myservo;                                      // create servo object to control a servo 
 
int pos=0;                                          // variable to store the servo position
int URPWM=3;                                        // PWM Output 0-25000us,every 50us represent 1cm
int URTRIG=5;                                       // PWM trigger pin
boolean up=true;                                    // create a boolean variable
unsigned long time;                                 // create a time variable
unsigned long urmTimer = 0;                          // timer for managing the sensor reading flash rate
 
unsigned int Distance=0;
uint8_t EnPwmCmd[4]={0x44,0x22,0xbb,0x01};          // distance measure command

const char LEFT_SENSOR = 2;//2
const char MID_SENSOR = 1;
const char RIGHT_SENSOR = 0;//0
const char LEFTSIDE_SENSOR = 3;
const char RIGHTSIDE_SENSOR = 4;//0

const double P_FAST = 0.01;
const double I_FAST = 0.045;
double adjustment;

Motors motors;
SingleWheelEncoders swe;

const int NUM_SENSOR_READINGS = 5;

String content="";  //read serial input
char character;       //store the serial input one by one
char done=true;


void setup() {
  Serial.begin(115200);
  //Serial.println("Dual VNH5019 Motor Shield");
  motors.init(13,11);
  myservo.attach(9);                                // Pin 9 to control servo

}

void loop() {
  int distLeft = readLeftIR()/10;
  if(distLeft>9)
  {
    distLeft=-1;
  }
  int distMid = readMidIR()/10;
  if(distMid>9)
  {
    distMid=-1;
  }
  int distRight = readRightIR()/10;
  if(distRight>9)
  {
    distRight=-1;
  }
  

  int distRightSide = readRightSideIR()/10;
  if(distRightSide>9)
  {
    distRightSide=-1;
  }
  
  int distLeftSide = readLeftSideIR()/10;
  if(distLeftSide>9)
  {
    distLeftSide=-1;
  }
  
  if(done==true)
  {
    Serial.print(distLeft);
    Serial.print(",");
    Serial.print(distMid);
    Serial.print(",");
    Serial.print(distRight);
    Serial.print(",");
    Serial.print(distLeftSide);
    Serial.print(",");
    Serial.print(distRightSide);
    Serial.print(",");
    Serial.println();
    Serial.flush();
    done=false;
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
            done=true;
            Serial.println("OK");
            break;
          case 'l':
          case 'L': //turn left
            if (content.charAt(3) == '1'){
             motors.turnLeftFast();
             done=true;
             Serial.println("OK");
           }
           else{
              motors.turnLeft();
              done=true;
              Serial.println("OK");
           }
            break;
          case 'r':
          case 'R': //turn right
            if (content.charAt(3) == '1'){
             motors.turnRightFast();
             done=true;
             Serial.println("OK");
           }else{             
            motors.turnRight();
            done=true;
            Serial.println("OK");
           }
            break;
          case 'b':
          case 'B':
            blocks = content.substring(1,3).toInt();
            motors.moveBackward(blocks);
            done=true;
            Serial.println("OK");
            break;
//          case 'a':
//          case 'A':
//            adjustCorner();
//            break;
          
            
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
       }  
}

//Method to read various sensor, each read is about 39ms
//which will take 390ms, will investigate further to lower this delay
float readLeftIR() {
  int adcValue = readSensor(LEFT_SENSOR);
  //error condition or undetectable range
  if(adcValue == -1)
    return -1;
  //return ((-2.0718*pow(10,-6)*pow(adcValue,3))+(0.00226234*pow(adcValue,2))-(0.808266*adcValue)+114.864);
  //return(12343.85 * pow(adcValue,-1.15));  
  return (pow(3027.4 / adcValue, 1.2134)); 
  //return (10106*pow(adcValue, -1.12))-5.7;//dist;//left
}

int readMidIR() {
  int adcValue = readSensor(MID_SENSOR);
  //error condition or undetectable range
  if(adcValue == -1)
    return -1;
  //return 9462/(adcValue - 16.92);
  //return ((-2.0718*pow(10,-6)*pow(adcValue,3))+(0.00226234*pow(adcValue,2))-(0.808266*adcValue)+114.864);
  //return(12343.85 * pow(adcValue,-1.15));  
  return (pow(3027.4 / adcValue, 1.2134));  
  //return (10106*pow(adcValue, -1.12));//56

}

int readRightSideIR() {
  int adcValue = readSensor(RIGHTSIDE_SENSOR);
  //error condition or undetectable range
  if(adcValue == -1)
    return -1;
  //return ((-2.0718*pow(10,-6)*pow(adcValue,3))+(0.00226234*pow(adcValue,2))-(0.808266*adcValue)+114.864);
  //return(12343.85 * pow(adcValue,-1.15));  
  return (pow(3027.4 / adcValue, 1.2134));  
  //return (10106*pow(adcValue, -1.12));//56
}

int readLeftSideIR() {
  int adcValue = readSensor(LEFTSIDE_SENSOR);
  //error condition or undetectable range
  if(adcValue == -1)
    return -1;
  //return ((-2.0718*pow(10,-6)*pow(adcValue,3))+(0.00226234*pow(adcValue,2))-(0.808266*adcValue)+114.864);
  //return(12343.85 * pow(adcValue,-1.15));  
  return (pow(3027.4 / adcValue, 1.2134));    
  //return (10106*pow(adcValue, -1.12));//56
}

int readRightIR() {
  int adcValue = readSensor(RIGHT_SENSOR);
  //error condition or undetectable range
  if(adcValue == -1)
    return -1;
 //return ((-2.0718*pow(10,-6)*pow(adcValue,3))+(0.00226234*pow(adcValue,2))-(0.808266*adcValue)+114.864);
 //return(12343.85 * pow(adcValue,-1.15));  
  return (pow(3027.4 / adcValue, 1.2134));  
  //return (10106*pow(adcValue, -1.12));
  //return 1/((0.0003*adcValue) - 0.0236);

}

int readSensor(char sensorPin) {
  char i;
  long sensorReadings = 0;
  int adcValues[10];
  for(i=0; i<10; i++) {
    sensorReadings = analogRead(sensorPin);
    adcValues[i] = sensorReadings;//analogRead(sensorPin);
    delay(35);
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
