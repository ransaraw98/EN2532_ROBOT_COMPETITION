/*
 * File:          dashed_line.c
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

/*
 * You may want to add macros here.
 */
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <stdio.h>
#include <webots/inertial_unit.h>
#include <webots/camera.h>
#include <webots/camera.h>
#include <stdlib.h>
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
double qtrStore[7][7];
char QTR_names[nQTR][6] = { "qtr0","qtr1","qtr2","qtr3","qtr4","qtr5","qtr6","qtr7" };
char wheels_names[2][12] = { "right_motor", "left_motor" };
double error[8] = { 0,0,0,0,0,0,0,0 };
double weights[8] = { 0,-2.7,-1.3,-0.8,0,0.8,1.3,2.7};
double Kp = 0.15;
double Kd = 0.04;                                                          
double P = 0;
double D = 0;
double pEr = 0;
double baseSpeed = 400;  //  rad/s
double Speeds[2] = { 0,0 };
double PID = 0;
short junc = -1;
unsigned short int state = 0;
char ds_names[3][10] = { "front_ir","left_ir","right_ir" };
unsigned short quardrant = 0;
unsigned char path;

int line_follow = 0;
int wall_flag = 0;
int state4 = 0;
WbDeviceTag QTR[nQTR];
WbDeviceTag wheels[2];
WbDeviceTag IMU;
WbDeviceTag ds[3];

/*Color detection*/
unsigned char red;
unsigned char green;
unsigned char blue;

WbDeviceTag CAM1;
WbDeviceTag CAM2;

/*  ARM   */
WbDeviceTag lr_1;
WbDeviceTag lr_2;
WbDeviceTag Sr_1;
WbDeviceTag Sr_2;

double S1 = 0.0;
double S2 = 3.12;
double linear = 0.0;

void swap(double* xp, double* yp)
{
    double temp = *xp;
    *xp = *yp;
    *yp = temp;
}

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

void ReadQTR2(WbDeviceTag QTRa[]) {

    for (int i = 0; i < nQTR - 1; i++) {

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

    for (int i = 0; i < 7; i++) {
        error[i] = qtrStore[i][3] - 250;

    }


}

void lineFollow2(double coeff) {
    if (line_follow == 1) {
        double eSUM2 = 0;
        for (int i = 0; i < 7; i++) { //neglect the leftmost sensor
            eSUM2 += error[i] * weights[i + 1];
        }
        P = Kp * eSUM2;
        D = Kd * (eSUM2 - pEr);
        PID = P + D;
        Speeds[0] = baseSpeed / coeff + PID;
        Speeds[1] = baseSpeed / coeff - PID;
        pEr = eSUM2;
        printf("%f\n", PID);

        wb_motor_set_position(wheels[0], INFINITY);
        wb_motor_set_velocity(wheels[0], 0.00628 * Speeds[0]);
        wb_motor_set_position(wheels[1], INFINITY);
        wb_motor_set_velocity(wheels[1], 0.00628 * Speeds[1]);
    }
}


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
   /*Initialize Motors*/

    for (int i = 0; i < 2; i++)
    {
        wheels[i] = wb_robot_get_device(wheels_names[i]);
        wb_motor_set_position(wheels[i], INFINITY);
        wb_motor_set_velocity(wheels[i], 0);
    }
    
    /*Initialize distance sensors*/

    for (int i = 0; i < 3; i++) {

        ds[i] = wb_robot_get_device(ds_names[i]);
        wb_distance_sensor_enable(ds[i], TIME_STEP);
        printf("Initialized ds %d\n", i);
    }
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

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
        ReadQTR2(QTR);
        lineFollow2(1);
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
