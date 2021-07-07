
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <webots/DistanceSensor.hpp>


#define TIME_STEP 64
#define nQTR 8
using namespace webots;
unsigned int qtrValues[8] ={0,0,0,0,0,0,0,0};
Robot *robot = new Robot();
DistanceSensor *QTR[nQTR];
Motor *wheels[2];
char QTR_names[nQTR][6]= {"qtr0","qtr1","qtr2","qtr3","qtr4","qtr5","qtr6","qtr7"};
char wheels_names[2][12] = {"right_motor", "left_motor"};
bool lineFollow = 0;




void readQTR(){
  for (int i=0;i<8;i++){
    qtrValues[i] = QTR[i]->getValue();         
  }
}




int main(int argc,char **argv) {

  

  
  /////////////////////////////INITIALIZING QTR ARRAY///////////////////////////////
  for(int i = 0; i < 2; i++) {
    QTR[i] = robot->getDistanceSensor(QTR_names[i]);
    QTR[i]->enable(TIME_STEP);
  }
  

//////////////////////////INITIALIZING DRIVE MOTORS///////////////////////////////
  for (int i = 0; i < 2; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
}
  

/////////////////////////////////////ARM/////////////////////////////////////////
  
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
  readQTR();
      printf("%4d   %4d    %4d    %4d    %4d    %4d    %4d    %4d\n",qtrValues[0],qtrValues[1],qtrValues[2],qtrValues[3],qtrValues[4],qtrValues[5],qtrValues[6],qtrValues[7]);
  /////////////////////////////////////ARM/////////////////////////////////////////
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