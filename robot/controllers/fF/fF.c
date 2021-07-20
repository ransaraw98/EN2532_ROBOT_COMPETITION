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
#include <webots/inertial_unit.h>
#include <webots/camera.h>
  ////////////////////////////////////// * MACROS ///////////////////////////////////////////////

#define TIME_STEP 16
/* Line Follow */
#define nQTR 8
#define MAX_GS 880 //840
#define MIN_GS 40
#define NEW_GS 1000 
#define BUFLEN 7
#define MID 3
////////////////////////////////////// *VARIABLES ///////////////////////////////////////////////
/* QTR SENSOR ARRAY + LINE FOLLOW INITIALIZATIONS*/
double qtrValues[8] = { 0,0,0,0,0,0,0,0 };
double qtrNew[nQTR] = { 0, 0, 0, 0, 0,0,0,0 };
double maxGS[nQTR] = { 500, 500, 500, 500, 500, 500, 500, 500 };
double minGS[nQTR] = { 500, 500, 500, 500, 500, 500, 500, 500 };
double qtrStore[7][5];
char QTR_names[nQTR][6] = { "qtr0","qtr1","qtr2","qtr3","qtr4","qtr5","qtr6","qtr7" };
char wheels_names[2][12] = { "right_motor", "left_motor" };
double error[8] = { 0,0,0,0,0,0,0,0 };
double weights[8] = { 0,5,1.3,1,0,-1,-1.3,-5};
double Kp = 0.45;
double Kd = 0.1;
double P = 0;
double D = 0;
double pEr = 0;
double baseSpeed = 200;  //  rad/s
double Speeds[2] = { 0,0 };
double PID = 0;
short hardLeft, hardRight, Tjunc = 0;

WbDeviceTag QTR[nQTR];
WbDeviceTag wheels[2];
WbDeviceTag IMU;

/*Color detection*/
unsigned char red;
unsigned char green;
unsigned char blue;

WbDeviceTag CAM1;
WbDeviceTag CAM2;

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
void swap(double* xp, double* yp);
void selectionSort(double arr[], int n);
void ReadQTR2(WbDeviceTag QTRa[]);
//void hardLeftf(void);


void swap(double* xp, double* yp)
{
    double temp = *xp;
    *xp = *yp;
    *yp = temp;
}

// Function to perform Selection Sort
void selectionSort(double arr[], int n)
{
    int i, j, min_idx;

    // One by one move boundary of unsorted subarray
    for (i = 0; i < n - 1; i++) {

        // Find the minimum element in unsorted array
        min_idx = i;
        for (j = i + 1; j < n; j++)
            if (arr[j] < arr[min_idx])
                min_idx = j;

        // Swap the found minimum element
        // with the first element
        swap(&arr[min_idx], &arr[i]);
    }
}



void ReadQTR(WbDeviceTag *QTRa) {
    /*
    Read all 7 IR distance sensors and update their values in qtrNew array
    update error array
    set hardLeft hardRight flags
    */
    for (int i = 0; i < nQTR; i++) {
        qtrValues[i] = wb_distance_sensor_get_value(QTRa[i]);

        //    Max & Min detection
        if (qtrValues[i] < minGS[i]) minGS[i] = qtrValues[i];
        if (qtrValues[i] > maxGS[i]) maxGS[i] = qtrValues[i];

        // linear Interpolation
        qtrNew[i] = ((double)qtrValues[i] - minGS[i]) / (maxGS[i] - MIN_GS) * NEW_GS;

        // Limited values between 0 and 1000 (NEW_GS)
        if (qtrNew[i] > NEW_GS) qtrNew[i] = NEW_GS;
        if (qtrNew[i] < 0) qtrNew[i] = 0;

        /*if (qtrNew[i] < 70) {
            error[i] = 0;
        }
        else {
            error[i] = 1000;//qtrNew[i] - 70;
        }
        */
        error[i] = qtrNew[i] - 70;
    }
    
     if ((error[1] == 0) && (error[2] == 0) && (error[3] == 0) && (error[4] == 0) && (error[5] == 0) && (error[6] == 1000) && (error[7] == 1000))  {

          hardLeft = 1;
          printf("hardleft");
      }
      else {
          hardLeft = 0;
      }
      if ((error[1] == 1000) && (error[2] == 1000) && (error[3] == 0) && (error[4] == 0) && (error[5] == 0) && (error[6] == 0) && (error[7] == 0)) {

          hardRight = 1;

      }
      else {
          hardRight = 0;
      }
 

}


void ReadQTR2(WbDeviceTag QTRa[]) {

    for (int i = 0; i < nQTR-1;i++) {

        qtrStore[0][i] = wb_distance_sensor_get_value(QTRa[1]); // neglect leftmost sensor
        qtrStore[1][i] = wb_distance_sensor_get_value(QTRa[2]);
        qtrStore[2][i] = wb_distance_sensor_get_value(QTRa[3]);
        qtrStore[3][i] = wb_distance_sensor_get_value(QTRa[4]);
        qtrStore[4][i] = wb_distance_sensor_get_value(QTRa[5]);
        qtrStore[5][i] = wb_distance_sensor_get_value(QTRa[6]);
        qtrStore[6][i] = wb_distance_sensor_get_value(QTRa[7]);
    
    }
    selectionSort(qtrStore[0], BUFLEN);
    selectionSort(qtrStore[1], BUFLEN);
    selectionSort(qtrStore[2], BUFLEN);
    selectionSort(qtrStore[3], BUFLEN);
    selectionSort(qtrStore[4], BUFLEN);
    selectionSort(qtrStore[5], BUFLEN);
    selectionSort(qtrStore[6], BUFLEN);

    for (int i = 0; i < 7;i++) {
        error[i] = qtrStore[i][3]-250;
   
    }
    if ((error[0] < 350) && (error[1] < 350) && (error[2] < 350) && (error[3] < 350) && (error[4] < 350) && (error[5] > 400) && (error[6] > 400)) {

        hardLeft = 1;
        printf("hardleft");
    }
    else {
        hardLeft = 0;
    }

}
void hardLeftf(void) {
    const double initAngle = wb_inertial_unit_get_roll_pitch_yaw(IMU)[2];
    printf("HARDLEFT\n");
    while (wb_inertial_unit_get_roll_pitch_yaw(IMU)[2]-initAngle< 1.57) {
        wb_motor_set_velocity(wheels[0], 3);
        wb_motor_set_velocity(wheels[1], -0.25);
        wb_robot_step(TIME_STEP);
    }
    hardLeft = 0;
 
}
void hardRightf(void) {
    const double initAngle = wb_inertial_unit_get_roll_pitch_yaw(IMU)[2];
    printf("HARDRIGHT\n");
    while (wb_inertial_unit_get_roll_pitch_yaw(IMU)[2]-initAngle > -1.57) {
        wb_motor_set_velocity(wheels[0], 0);
        wb_motor_set_velocity(wheels[1], 3);
        wb_robot_step(TIME_STEP);
    }
    hardLeft = 0;

}


void lineFollow(void) {
    double eSUM = 0;

    for (int i = 1; i < 8; i++) { //neglect the leftmost sensor
        eSUM += error[i] * weights[i];
    }

    P = Kp * eSUM;
    D = Kd * (eSUM - pEr);
    PID = P + D;
    Speeds[0] = baseSpeed - PID;
    Speeds[1] = baseSpeed + PID;
    pEr = eSUM;
}

void lineFollow2(void) {
    double eSUM2 = 0;
    for (int i = 0; i < 7; i++) { //neglect the leftmost sensor
        eSUM2 += error[i] * weights[i+1];
    }
    P = Kp * eSUM2;
    D = Kd * (eSUM2 - pEr);
    PID = P + D;
    Speeds[0] = baseSpeed - PID;
    Speeds[1] = baseSpeed + PID;
    pEr = eSUM2;
    printf("%f\n",PID);

}

unsigned char readColor(WbDeviceTag camera) {
    /*
    Returns the color detected as a char 'R','G','B'
    the color test is performed in the given order
    assumes the object is ONLY one of the three colors
    for example white will be detected as R in here.


    Input should be an initialized WbDeviceTag device


    */
    unsigned char R = 'R';
    unsigned char G = 'G';
    unsigned char B = 'B';

    unsigned char* image = wb_camera_get_image(camera);

    if (wb_camera_image_get_red(image, 8, 4, 4) > 200) {
        return R;
    }
    else if (wb_camera_image_get_green(image, 8, 4, 4) > 200) {
        return G;
    }
    else if (wb_camera_image_get_blue(image, 8, 4, 4) > 200) {
        return B;
    }

    else {
        return 0;
    }
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char** argv) {
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

    for (int i = 0; i < 2; i++)
    {
        wheels[i] = wb_robot_get_device(wheels_names[i]);
        wb_motor_set_position(wheels[i], INFINITY);
        wb_motor_set_velocity(wheels[i], 0);
    }

    /*Initialize IMU */

    IMU = wb_robot_get_device("inertial_unit");
    wb_inertial_unit_enable(IMU, TIME_STEP); 


    /* Initialize cameras*/

    CAM1 = wb_robot_get_device("camera1");
    CAM2 = wb_robot_get_device("camera2");
    wb_camera_enable(CAM1, TIME_STEP);
    wb_camera_enable(CAM2, TIME_STEP);


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
        ReadQTR2(QTR);
        lineFollow2();
        if (hardLeft == 1) {
            hardLeftf();
        }
        if (hardRight == 1) {
            hardRightf();
        }
        wb_motor_set_position(wheels[0], INFINITY);
        wb_motor_set_velocity(wheels[0], 0.00628 * Speeds[0]);
        wb_motor_set_position(wheels[1], INFINITY);
        wb_motor_set_velocity(wheels[1], 0.00628 * Speeds[1]);
        //printf("%4f   %4f   %4f   %4f    %4f    %4f   %4f    %4f", qtrNew[0], qtrNew[1], qtrNew[2], qtrNew[3], qtrNew[4], qtrNew[5], qtrNew[6], qtrNew[7]);
        printf("%4f   %4f   %4f   %4f    %4f    %4f   %4f    %4f", error[0], error[1], error[2], error[3], error[4], error[5], error[6], error[7]);
        /* Process sensor data here */
       
        printf("\t CAM1  %c \t CAM2  %c\n",readColor(CAM1),readColor(CAM2));
        
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
