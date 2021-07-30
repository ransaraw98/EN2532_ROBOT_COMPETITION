#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#define TIME_STEP 16
using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  DistanceSensor *ds[2];
  char dsNames[2][11] = {"ds_fright", "ds_fleft"};
  for (int i = 0; i < 2; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }
  Motor *wheels[2];
  char wheels_names[2][9] = {"Motor_1", "Motor_2"};
  for (int i = 0; i < 2; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  int avoidObstacleCounter = 0;
  double ls;
  double rs;
  while (robot->step(TIME_STEP) != -1) {
    double leftSpeed = -4.0;
    double rightSpeed = -4.0;
    if (avoidObstacleCounter > 0) {
      avoidObstacleCounter--;
      leftSpeed = ls;
      rightSpeed = rs;
    } else { // read sensors
      
        if ((ds[0]->getValue() < 1000.0) && (ds[0]->getValue() < ds[1]->getValue()) ){
	  ls = 1.5;
	  rs = -1.5;
             avoidObstacleCounter = 10;
          }
	else if ((ds[1]->getValue() < 1000.0) && (ds[0]->getValue() > ds[1]->getValue())){
	  ls = -1.5;
	  rs = 1.5;
             avoidObstacleCounter = 10;
          }	
	else if ((ds[0]->getValue() < 1000.0) || (ds[1]->getValue() < 1000.0)){
	  avoidObstacleCounter = 10;
          }
	  	  	
    }
    wheels[0]->setVelocity(rightSpeed);
    wheels[1]->setVelocity(leftSpeed);
    
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}