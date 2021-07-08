// File:          wall_follow.cpp
// Date:          8/7/2021
// Description:
// Author:        Vidushika Rasanji
// Modifications:

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#define TIME_STEP 64
using namespace std;
using namespace webots;


int main(int argc, char **argv) {
  
  Robot *robot = new Robot();
  DistanceSensor *ds[2];
  char dsNames[2][10] = {"left_ir","right_ir"};
  for (int i = 0; i < 2; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }
  Motor *wheels[2];
  char wheels_names[2][12] = {"left_motor", "right_motor"};
  for (int i = 0; i < 2; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  
  

  while (robot->step(TIME_STEP) != -1) {
  double leftSpeed = 1.0;
  double rightSpeed = 1.0;
  double vall=ds[0]->getValue();
  double valr=ds[1]->getValue();
   if (valr<1000){
         if (valr>690){
                leftSpeed = 1.0;
                rightSpeed = -1.0;
               }
          else if (valr<570){
                leftSpeed = -1.0;
                rightSpeed = 1.0;
               }}
   
   else if (vall<1000){
         if (vall>690){
                leftSpeed =-1.0;
                rightSpeed =1.0;
               }
          else if (vall<570){
                leftSpeed =1.0;
                rightSpeed =-1.0;
               }}
  wheels[0]->setVelocity(leftSpeed);
  wheels[1]->setVelocity(rightSpeed);  
  }

  
  delete robot;
  return 0;
}
