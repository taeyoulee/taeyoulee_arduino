#include "DualVNH5019MotorShield.h"
#include "SingleWheelEncoders.h"
#include "PID_v1.h"
#include "Motors.h"
#include "PinChangeInt.h"
#include <cstring.h>

Motors motors;
SingleWheelEncoders swe;

void setup()
{
  Serial.begin(115200);
  //motors.init(13,11);
  motors.init(11,13);
}

void loop() {
  for (int i=0; i<1; i++){
    //moveForward
    motors.moveForward(10); 
    
    //rotation
    //motors.rotate(60, false);
    
 
    //other functions
    //motors.moveForward(10);
    //delay(1000);
    //motors.turnRight();
    //delay(1000);
    //motors.moveForward(1);   
    
    //motors.turnLeftFast();
    //motors.turnRight();
    //motors.turnRightFast();
    //motors.rotate(850, false);
    //motors.turnLeft(); 
    //motors.moveForward(10);
    //delay(1000);
  }
  while(1);
}


