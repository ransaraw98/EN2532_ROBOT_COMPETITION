/*
 * File:          lineFollowTest.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <stdio.h>
#include <webots/gyro.h>
// * You may want to add macros here.

#define TIME_STEP 16
#define nQTR 8
#define MAX_GS 880 //840
#define MIN_GS 300
#define NEW_GS 1000 


/* QTR SENSOR ARRAY + LINE FOLLOW INITIALIZATIONS*/
unsigned int qtrValues[8] = { 0,0,0,0,0,0,0,0 };
short qtrNew[nQTR] = {0, 0, 0, 0, 0,0,0,0};
unsigned short maxGS[nQTR]= {500, 500, 500, 500, 500, 500, 500, 500};
unsigned short minGS[nQTR]= {500, 500, 500, 500, 500, 500, 500, 500};
unsigned long Position = 0;
char QTR_names[nQTR][6] = { "qtr0","qtr1","qtr2","qtr3","qtr4","qtr5","qtr6","qtr7" };
char wheels_names[2][12] = { "right_motor", "left_motor" };
unsigned short error[8] = { 0,0,0,0,0,0,0,0 };
float weights[8] = { 0,2.5,1.1,0.5,0,-0.5,-1.1,-2.5 };
float Kp = 0.1;
float Kd = 0.04;
float P = 0;
float D = 0;
float pEr = 0;
float baseSpeed = 400;  //  rad/s
float Speeds[2] = {0,0};
float PID = 0;
bool hardLeft,hardRight,Tjunc = 0;
//void readQTR(WbDeviceTag* QTRarray); //function prototype

WbDeviceTag QTR[nQTR];
WbDeviceTag wheels[2];
WbDeviceTag GYRO;
/*
void readQTR(WbDeviceTag *QTRarray) {
    for (int i = 0; i < 8; i++) {
        qtrValues[i] = wb_distance_sensor_get_value(QTRarray[i]);
    }
}
*/

/* Function PROTOTYPES */
void ReadQTR(WbDeviceTag* QTRa); 
void lineFollow(void);



 
void ReadQTR(WbDeviceTag *QTRa){
    
  for(int i=0; i<nQTR; i++){
    qtrValues[i] = wb_distance_sensor_get_value(QTRa[i]);

//    Max & Min detection
    if(qtrValues[i]<minGS[i]) minGS[i]=qtrValues[i];
    if(qtrValues[i]>maxGS[i]) maxGS[i]=qtrValues[i];
    
    // linear Interpolation
    qtrNew[i] = ((float)qtrValues[i]-MIN_GS)/(MAX_GS-MIN_GS)*NEW_GS;

    // Limited values between 0 and 1000 (NEW_GS)
    if(qtrNew[i]>NEW_GS) qtrNew[i]=NEW_GS;
    if(qtrNew[i]<0) qtrNew[i]=0;
    
    if (qtrNew[i] < 70) {
        error[i] = 0;
    }
    else{
        error[i] = 1000;//qtrNew[i] - 70;
    }
  }
  if((error[1]==0)&&(error[2]==0)&&(error[3]==0)&&(error[4]==0)&&(error[5]==0)&&(error[6]==1000)&&(error[7]==1000)){
  
  hardLeft =1;
  
  }
  else{
  hardLeft =0;
  }
  if((error[1]==1000)&&(error[2]==1000)&&(error[3]==0)&&(error[4]==0)&&(error[5]==0)&&(error[6]==0)&&(error[7]==0)){
  
  hardRight =1;
  
  }
  
  
}



void hardLeftf (void){
//double *currentW[3];


if(hardLeft){
double totalAngle=0;
printf("HARDLEFT\n");
wb_gyro_enable(GYRO,10);
while (totalAngle<10){
totalAngle+=  wb_gyro_get_values(GYRO)[1]*0.016;
printf("angle %f\n",totalAngle);
wb_motor_set_velocity(wheels[0], 0.5);
wb_motor_set_velocity(wheels[1], 0);

}

}

}

void lineFollow(void) {
    float eSUM = 0;

    for (int i = 1; i < 8; i++) { //neglect the leftmost sensor
        eSUM += error[i] * weights[i];
    }

    P = Kp * eSUM;
    D = Kd * (eSUM - pEr);
    PID = P + D;
    Speeds[0] = baseSpeed - PID;
    Speeds[1] = baseSpeed + PID;
    pEr=eSUM;
}


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  
  /*Initialize QTR Array*/
  for (int i = 0; i < 8; i++) {
      QTR[i] = wb_robot_get_device(QTR_names[i]);
      wb_distance_sensor_enable(QTR[i], TIME_STEP);
  }

  /*Initialize Motors*/

  for (int i = 0; i <2 ; i++)
  {
      wheels[i] = wb_robot_get_device(wheels_names[i]);
      wb_motor_set_position(wheels[i],INFINITY);
      wb_motor_set_velocity(wheels[i], 0);
  }
  
  /*Initialize Gyro */
  
  GYRO = wb_robot_get_device("gyro");
  
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

      //readQTR(QTR);
      ReadQTR(QTR);
      lineFollow();
      hardLeftf();
      wb_motor_set_velocity(wheels[0], 0.00628 * Speeds[0]);
      wb_motor_set_velocity(wheels[1], 0.00628 * Speeds[1]);
      printf("%4d   %4d    %4d    %4d    %4d    %4d    %4d    %4d\n", qtrNew[0], qtrNew[1], qtrNew[2], qtrNew[3], qtrNew[4], qtrNew[5], qtrNew[6], qtrNew[7]);
    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
