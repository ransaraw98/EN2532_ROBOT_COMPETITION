
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>



#define TIME_STEP 64
using namespace webots;

int main() {
  Robot *robot = new Robot();
  Keyboard kb;
 
  Motor *Sr_1;
  Motor *Sr_2;
  Motor *lr_1;
  Motor *lr_2;
  Sr_1=robot->getMotor("Servo1");
  Sr_2=robot->getMotor("Servo2");
  lr_1=robot->getMotor("linear1");
  lr_2=robot->getMotor("linear2");

  kb.enable(TIME_STEP);
  double S1=0.0;
  double S2=0.0;
  double linear=0.0;

  while (robot->step(TIME_STEP) != -1) {
    int key=kb.getKey();
  
    // std::cout<<ds[0]->getValue()<<"=Right Sensor"<<std::endl;
    // std::cout<<ds[1]->getValue()<<"=Left Sensor"<<std::endl;

    if (key==87 && S1<0.2){//W and S
    S1 += 0.1;
    } else if (key==83 && S1>-2){
    S1 += -0.1;
    }else {
    S1+=0;
    }
    if (key==68 && S2<3.14){//A and D
    S2 += 0.1;
    } else if (key==65 && S2>0){
    S2 += -0.1;
    }else {
    S2+=0;
    }
    
    if (key==74 && linear<3.98986e-17){//J and L
    linear += 0.005;
    } else if (key==76 && linear>-0.03){
    linear += -0.005;
    }else {
    linear+=0;
    }
    
    Sr_1->setPosition(S1);
    Sr_2->setPosition(S2);
    lr_1->setPosition(-linear);
    lr_2->setPosition(linear);
    // std::cout<<key<<std::endl;
    
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}