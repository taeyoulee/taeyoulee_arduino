#include "DualVNH5019MotorShield.h"
#include "SingleWheelEncoders.h"
#include "PID_v1.h"
#include "Motors.h"
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

Motors motors;
SingleWheelEncoders swe;

const int NUM_SENSOR_READINGS = 5;

String content="";  //read serial input
char character;       //store the serial input one by one

void setup()
{
  Serial.begin(115200);
  //motors.init(13,11);
  motors.init(11,13);
}

void loop() {
  for (int i=0; i<100; i++){
      //Serial.print("Sensor left: ");
    int distLeft = readLeftIR()/10;
    if(distLeft>9)
    {
      distLeft=-1;
    }
    //Serial.println(distLeft);
    //delay(200);
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
    //Serial.print(distLeft);
    //Serial.print(",");
    //Serial.print(distMid);
    //Serial.print(",");
    //Serial.print(distRight);
    //Serial.print(",");
    //Serial.print(distLeftSide);
    //Serial.print(",");
    //Serial.print(distRightSide);
    //Serial.print(",");
    //Serial.println();
    //Serial.println("--------");
    //writeToRPI(distLeft, distMid, distRight);
   
    Serial.println(distMid);
    
    if(distMid <= 1 && distMid >=0 ){
      
      motors.turnLeft();
      delay(100);
      motors.roundAbout(3100, false, false, 80.0, 80.3);
      delay(100);
      motors.rotate(77, true);
      delay(100); 
    }else{
      motors.moveForward(1);

    }
    //moveForward
    //motors.moveForward(5); 
    
    //rotation
    //motors.rotate(60, false);
    
 
    //other functions
    //motors.moveForward(10);
    //delay(200);
    //motors.turnRight();
    //delay(200);
    //motors.moveForward(1);   
    
    //motors.turnLeftFast();
    //motors.turnRight();
    //motors.turnRightFast();
    //motors.rotate(850, false);
    //motors.turnLeft(); 
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
    //delay(35);
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

void writeToRPI(int distLeft, int distMid, int distRight) {
  char obstacles[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}, i;

  if(30 <= distLeft && distLeft < 40) {//furtherest obs
    obstacles[0] = '0';
    obstacles[3] = '0';
    obstacles[6] = '1';
  }
  else if(20 <= distLeft && distLeft < 30) {
    obstacles[0] = '0';
    obstacles[3] = '1';
    obstacles[6] = '0';
  }
  else if(9 <= distLeft && distLeft < 20) {//nearest obs
    obstacles[0] = '1';
    obstacles[3] = '0';
    obstacles[6] = '0';
  }
  
  
  if(30 <= distMid && distMid < 40) {//furtherest obs
    obstacles[1] = '0';
    obstacles[4] = '0';
    obstacles[7] = '1';
  }
  else if(20 <= distMid && distMid < 30) {
    obstacles[1] = '0';
    obstacles[4] = '1';
    obstacles[7] = '0';
  }
  else if(9 <= distMid && distMid < 20) {//nearest obs
    obstacles[1] = '1';
    obstacles[4] = '0';
    obstacles[7] = '0';
  }
  
  
  if(30 <= distRight && distRight < 40) {//furtherest obs
    obstacles[2] = '0';
    obstacles[5] = '0';
    obstacles[8] = '1';
  }
  else if(20 <= distRight && distRight < 30) {
    obstacles[2] = '0';
    obstacles[5] = '1';
    obstacles[8] = '0';
  }
  else if(9 <= distRight && distRight < 20) {//nearest obs
    obstacles[2] = '1';
    obstacles[5] = '0';
    obstacles[8] = '0';
  }
  Serial.write("1,");
  for(i=0; i<9; i++) {
    Serial.write(obstacles[i]);
  }
  Serial.write(";\n");
}

//Converts the adc value passed as a parameter
//to centimetres mapped out using calibrated data
double convertToCM(int adcValue) {
  return floor(3059.1*pow((adcValue-30),-0.939));
}
