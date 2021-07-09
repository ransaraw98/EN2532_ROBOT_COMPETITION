#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>

#define TIME_STEP 64

void Lifting_box(double *linear,double *S1,double *S2,WbDeviceTag lr_1,WbDeviceTag lr_2,WbDeviceTag Sr_1,WbDeviceTag Sr_2){
 
 while(*linear>-0.02){
 *linear -= 0.001;
 wb_motor_set_position(lr_1, -*linear);
 wb_motor_set_position(lr_2, *linear);
 wb_robot_step(TIME_STEP);
}

while(*S1>-1){
 *S1 -= 0.04;
 wb_motor_set_position(Sr_1, *S1);
 wb_robot_step(TIME_STEP);
}
while(*S2<=3.11){
 *S2 += 0.04;
 wb_motor_set_position(Sr_2, *S2);
 wb_robot_step(TIME_STEP);
}

while(*S1>-2){
 *S1 -= 0.04;
 wb_motor_set_position(Sr_1, *S1);
 wb_robot_step(TIME_STEP);
}
printf("Servor 1 value is %f\n", *S1);
wb_robot_step(TIME_STEP);

}


void Placing_box(double *linear,double *S1,WbDeviceTag lr_1,WbDeviceTag lr_2,WbDeviceTag Sr_1){
printf("Servor 1 value is %f\n", *S1);
while(*S1<0){
 *S1 += 0.04;
 wb_motor_set_position(Sr_1, *S1);
 wb_robot_step(TIME_STEP);
}
while(*linear<=0.0){
 *linear += 0.001;
 wb_motor_set_position(lr_1, -*linear);
 wb_motor_set_position(lr_2, *linear);
 wb_robot_step(TIME_STEP);
}


wb_robot_step(TIME_STEP);

}



int main() {
  wb_robot_init();
  WbDeviceTag lr_1;
  WbDeviceTag lr_2;
  WbDeviceTag Sr_1;
  WbDeviceTag Sr_2;

  Sr_1=wb_robot_get_device("Servo1");
  Sr_2=wb_robot_get_device("Servo2");
  lr_1=wb_robot_get_device("linear1");
  lr_2=wb_robot_get_device("linear2");
  
  double S1=0.0;
  double S2=0.0;
  double linear=0.0;
  
  wb_motor_set_position(lr_1, -linear);
  wb_motor_set_position(lr_2, linear);
  wb_motor_set_position(Sr_2, S2);
  wb_motor_set_position(Sr_1, S1);
  
  Lifting_box(&linear,&S1,&S2,lr_1,lr_2,Sr_1,Sr_2);
  printf("Servor 1 value is %f\n", S1);
  Placing_box(&linear,&S1,lr_1,lr_2,Sr_1);
  while (wb_robot_step(TIME_STEP) != -1) {

  
  }
  wb_robot_cleanup();
  return 0;  // EXIT_SUCCESS

}
