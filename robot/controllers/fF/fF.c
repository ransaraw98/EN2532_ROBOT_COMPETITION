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
double qtrStore[7][5];
char QTR_names[nQTR][6] = { "qtr0","qtr1","qtr2","qtr3","qtr4","qtr5","qtr6","qtr7" };
char wheels_names[2][12] = { "right_motor", "left_motor" };
double error[8] = { 0,0,0,0,0,0,0,0 };
double weights[8] = { 0,-2.2,-1.5,-1,0,1,1.5,2.2};
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
char ds_names[3][10] = {"front_ir","left_ir","right_ir"};
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

        if (qtrNew[i] < 70) {
            error[i] = 0;
        }
        else {
            error[i] = 1000;//qtrNew[i] - 70;
        }
        
        //error[i] = qtrNew[i] - 70;
    }
    /*
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
 
 */
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


}

short j_check(void) {
    
    ReadQTR2(QTR);
    if ((error[0] < 350) && (error[1] < 350) && (error[2] < 350) && (error[3] < 350) && (error[4] < 350) && (error[5] > 400) && (error[6] > 400)) {
        printf("left junction found");
        return 1;
        
    }
    
    if ((error[0] > 400) && (error[1] > 400) && (error[2] < 350) && (error[3] < 350) && (error[4] < 350) && (error[5] < 350) && (error[6] < 350)) {

        printf("right junction found");
        return 2;
    }
   /* unsigned int flag = 1;
    for (int i = 0; i < 7; i++) {
        flag = flag && (error[i] < 100);
    if(flag) {
        return 3;
    }
    }
    */
    if ((error[0] < 100) && (error[1] < 100) && (error[2] < 100) && (error[3] < 100) && (error[4] < 100) && (error[5] < 100) && (error[6] < 100)) {

        printf("T junction found");
        return 3;
    }
    
    
    printf("NO JUNCT");
    return -1;
}
void hardLeftf(double angle, double RightSpeed, double LeftSpeed) {
    double initAngle = wb_inertial_unit_get_roll_pitch_yaw(IMU)[2];
    double readAngle = initAngle;
    double prevAngle = initAngle;
    double finalAngle = initAngle + angle;
    unsigned int Tflag = 0;
    //double delta = abs(finalAngle * 0.05);
    printf("HARDLEFT\n");
    printf("InitAngle = %f\n",initAngle);
    while (readAngle - initAngle < angle) {
        wb_motor_set_velocity(wheels[0], RightSpeed);
        wb_motor_set_velocity(wheels[1], LeftSpeed);
        readAngle = wb_inertial_unit_get_roll_pitch_yaw(IMU)[2];
        printf("read RAW = %f\t", readAngle);
        if (abs(prevAngle - readAngle) > 5) {
            Tflag = 1;
        }
        if (Tflag == 1) {
            if (initAngle < 0) {
                readAngle -= 6.28;
            }
            if (initAngle >= 0) {
                readAngle += 6.28;
            }
        }
        printf("read Modified = %f\n", readAngle);
        prevAngle = readAngle;
        wb_robot_step(TIME_STEP);
    }
    /*while (1) {
        readAngle  = wb_inertial_unit_get_roll_pitch_yaw(IMU)[2];
        wb_motor_set_velocity(wheels[0], 3);
        wb_motor_set_velocity(wheels[1], -0.25);
        if ((readAngle < finalAngle + delta) && (readAngle > finalAngle - delta)) { 
            break;
        }
        wb_robot_step(TIME_STEP);
    }*/
    junc = -1;
    for (int i = 0; i < 50; i++) {
        wb_motor_set_velocity(wheels[0], 0);
        wb_motor_set_velocity(wheels[1], 0);
        wb_robot_step(TIME_STEP);
    }
    
}
void hardRightf(double angle,double RightSpeed, double LeftSpeed) {
    double initAngle = wb_inertial_unit_get_roll_pitch_yaw(IMU)[2];
    double readAngle = initAngle;
    double prevAngle = initAngle;
    double finalAngle = initAngle + angle;
    unsigned int Tflag = 0;
    //double delta = abs(finalAngle * 0.05);
    printf("HARDRIGHT\n");
    printf("InitAngle = %f\n", initAngle);
    while (readAngle - initAngle > angle) {
        wb_motor_set_velocity(wheels[0], RightSpeed);
        wb_motor_set_velocity(wheels[1], LeftSpeed);
        readAngle = wb_inertial_unit_get_roll_pitch_yaw(IMU)[2];
        printf("read RAW = %f\t", readAngle);
        if (abs(prevAngle - readAngle) > 5) {
            Tflag = 1;
        }
        if (Tflag == 1) {
            if (initAngle < 0) {
                readAngle -= 6.28;
            }
            if (initAngle >= 0) {
                readAngle += 6.28;
            }
        }
        printf("read Modified = %f\n", readAngle);
        prevAngle = readAngle;
        wb_robot_step(TIME_STEP);
    }
    /*while (1) {
        readAngle  = wb_inertial_unit_get_roll_pitch_yaw(IMU)[2];
        wb_motor_set_velocity(wheels[0], 3);
        wb_motor_set_velocity(wheels[1], -0.25);
        if ((readAngle < finalAngle + delta) && (readAngle > finalAngle - delta)) {
            break;
        }
        wb_robot_step(TIME_STEP);
    }*/
    junc = -1;
    for (int i = 0; i < 50; i++) {
        wb_motor_set_velocity(wheels[0], 0);
        wb_motor_set_velocity(wheels[1], 0);
        wb_robot_step(TIME_STEP);
    }

}


void lineFollow(void) {
    double eSUM = 0;

    for (int i = 1; i < 8; i++) { //neglect the leftmost sensor
        eSUM += error[i] * weights[i];
    }
    printf("Total error %f", eSUM);
    P = Kp * eSUM;
    D = Kd * (eSUM - pEr);
    PID = P + D;
    Speeds[0] = baseSpeed + PID;
    Speeds[1] = baseSpeed - PID;
    pEr = eSUM;
}

void lineFollow2(double coeff) {
    if(line_follow == 1){ 
        double eSUM2 = 0;
        for (int i = 0; i < 7; i++) { //neglect the leftmost sensor
            eSUM2 += error[i] * weights[i + 1];
        }
        P = Kp * eSUM2;
        D = Kd * (eSUM2 - pEr);
        PID = P + D;
        Speeds[0] = baseSpeed/coeff + PID;
        Speeds[1] = baseSpeed/coeff - PID;
        pEr = eSUM2;
        printf("%f\n", PID);
    
        wb_motor_set_position(wheels[0], INFINITY);
        wb_motor_set_velocity(wheels[0], 0.00628 * Speeds[0]);
        wb_motor_set_position(wheels[1], INFINITY);
        wb_motor_set_velocity(wheels[1], 0.00628 * Speeds[1]);
    }
}

unsigned short readColor(WbDeviceTag camera) {
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
        return 1;
    }
    else if (wb_camera_image_get_green(image, 8, 4, 4) > 200) {
        return 2;
    }
    else if (wb_camera_image_get_blue(image, 8, 4, 4) > 200) {
        return 3;
    }

    else {
        return 0;
    }
}

/* wall follow */

void wall_follow(void) {

    if (wall_flag == 1) {
        
        double leftSpeed = 1.0;
        double rightSpeed = 1.0;
        double vall = wb_distance_sensor_get_value(ds[1]);
        double valr = wb_distance_sensor_get_value(ds[2]);
        printf("vall %f\n", vall);
        printf("valr %f\n", valr);
      
            printf("inside while\n");
            if (valr < 1000) {
                if (valr > 690) {
                    leftSpeed = 1.0;
                    rightSpeed = -1.0;
                }
                else if (valr < 570) {
                    leftSpeed = -1.0;
                    rightSpeed = 1.0;
                }
            }

            else if (vall < 1000) {

                if (vall > 690) {
                    leftSpeed = -1.0;
                    rightSpeed = 1.0;
                }
                else if (vall < 570) {
                    leftSpeed = 1.0;
                    rightSpeed = -1.0;
                }
                
            }
            wb_motor_set_position(wheels[0], INFINITY);
            wb_motor_set_velocity(wheels[0], rightSpeed);
            wb_motor_set_position(wheels[1], INFINITY);
            wb_motor_set_velocity(wheels[1], leftSpeed);

            vall = wb_distance_sensor_get_value(ds[1]);
            valr = wb_distance_sensor_get_value(ds[2]);
            printf("vall %f\n", vall);
            printf("valr %f\n", valr);
            wb_robot_step(TIME_STEP);
     
    
    }
   
    
}

/* Lifting Box*/

void Lifting_box(double* linear, double* S1, double* S2, WbDeviceTag lr_1, WbDeviceTag lr_2, WbDeviceTag Sr_1, WbDeviceTag Sr_2) {


    while (*S1 < 2.12) {
        *S1 += 0.04;
        wb_motor_set_position(Sr_1, *S1);
        wb_robot_step(TIME_STEP);
    }

    while (*linear > -0.02) {
        *linear -= 0.001;
        wb_motor_set_position(lr_1, -*linear);
        wb_motor_set_position(lr_2, *linear);
        wb_robot_step(TIME_STEP);
    }

    while (*S1 > 1) {
        *S1 -= 0.04;
        wb_motor_set_position(Sr_1, *S1);
        wb_robot_step(TIME_STEP);
    }

    while (*S2 >= 0) {
        *S2 -= 0.04;
        wb_motor_set_position(Sr_2, *S2);
        wb_robot_step(TIME_STEP);
    }

    while (*S1 >= 0.2) {
        *S1 -= 0.04;
        wb_motor_set_position(Sr_1, *S1);
        wb_robot_step(TIME_STEP);
    }
    printf("Servor 1 value is %f\n", *S1);
    wb_robot_step(TIME_STEP);

}

/* Placing box*/
void Placing_box(double* linear, double* S1, WbDeviceTag lr_1, WbDeviceTag lr_2, WbDeviceTag Sr_1) {
    printf("Servor 1 value is %f\n", *S1);
    while (*S1 < 2.12) {
        *S1 += 0.04;
        wb_motor_set_position(Sr_1, *S1);
        wb_robot_step(TIME_STEP);
    }
    while (*linear <= 0.0) {
        *linear += 0.001;
        wb_motor_set_position(lr_1, -*linear);
        wb_motor_set_position(lr_2, *linear);
        wb_robot_step(TIME_STEP);
    }

    while (*S1 >= 0.6) {
        *S1 -= 0.04;
        wb_motor_set_position(Sr_1, *S1);
        wb_robot_step(TIME_STEP);
    }

    wb_robot_step(TIME_STEP);

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

    /*Initialize distance sensors*/

    for (int i = 0; i < 3;i++   ) {
                                            
        ds[i] = wb_robot_get_device(ds_names[i]);
        wb_distance_sensor_enable(ds[i], TIME_STEP);
        printf("Initialized ds %d\n",i);
    }

    /*Initialize ARM*/

    Sr_1 = wb_robot_get_device("Servo1");
    Sr_2 = wb_robot_get_device("Servo2");
    lr_1 = wb_robot_get_device("linear1");
    lr_2 = wb_robot_get_device("linear2");

    wb_motor_set_position(lr_1, -linear);
    wb_motor_set_position(lr_2, linear);
    wb_motor_set_position(Sr_2, S2);
    wb_motor_set_position(Sr_1, S1);


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
        junc = j_check();
        printf("junc is %d\n", junc);
        if (state == 0) {
            if (junc == 3) {
                   line_follow = 1;
                   state++;
                }
            }
        

        if ((state == 1) && (wb_distance_sensor_get_value(ds[2]) < 1000)) {
            int i = 0;
            while (i < 5) {
                wb_motor_set_position(wheels[0], INFINITY);
                wb_motor_set_velocity(wheels[0], 0.00628 * Speeds[0]);
                wb_motor_set_position(wheels[1], INFINITY);
                wb_motor_set_velocity(wheels[1], 0.00628 * Speeds[1]);
                wb_robot_step(TIME_STEP);
                i++;
            }
            line_follow = 0;
            wall_flag = 1;
            state++;
        }
        if ((state == 2) && (wb_distance_sensor_get_value(ds[2]) >= 1000) && (wb_distance_sensor_get_value(ds[1]) >= 1000)) {
            for (int i = 0; i < 20; i++) {
                wb_motor_set_position(wheels[0], INFINITY);
                wb_motor_set_velocity(wheels[0], 0.00628 * Speeds[0]);
                wb_motor_set_position(wheels[1], INFINITY);
                wb_motor_set_velocity(wheels[1], 0.00628 * Speeds[1]);
                wb_robot_step(TIME_STEP);
            }
            line_follow = 1;
            wall_flag = 0;
            state++;
        }
        if ((state == 1)|| (state ==3)){
            
            if (junc == 1) {
                hardLeftf(1.25,2.5,-0.5);
            }
            if (junc == 2) {
                hardRightf(-1.47,-0.5,2.5);
                junc = -1;
            }

        }
        
        if ((state == 3)&& (junc ==3)) {

            hardRightf(-1.3,-0.5,2.5);
            line_follow = 1;
            Kp = 0.1;
            quardrant = 1;
            state++;

        }
        if (state == 4) {

            if (junc == 1) {
                line_follow = 0;
                hardLeftf(1.57,2.5,-0.5);
                state++;
                printf("STATE is %d \t LINE_FOLLOW is %d\n", state, line_follow);
            }
        }
            if (state == 5) {
                /*for (int i = 0; i < 100; i++) {
                    wb_motor_set_position(wheels[0], INFINITY);
                    wb_motor_set_velocity(wheels[0], -1);
                    wb_motor_set_position(wheels[1], INFINITY);
                    wb_motor_set_velocity(wheels[1], -1);
                    wb_robot_step(TIME_STEP);
                }
                */
                printf("BACKING");
                for (int i = 0; i < 120; i++) {
                    wb_motor_set_position(wheels[0], INFINITY);
                    wb_motor_set_velocity(wheels[0], -1);
                    wb_motor_set_position(wheels[1], INFINITY);
                    wb_motor_set_velocity(wheels[1], -1);
                    wb_robot_step(TIME_STEP);
                }

                /*for (int i = 0; i < 50; i++) {
                    line_follow = 1;
                    ReadQTR2(QTR);
                    lineFollow2(1);
                    wb_robot_step(TIME_STEP);
                    printf("calibration line follow\n");
                } */

                double temp = wb_distance_sensor_get_value(ds[0]);
                printf("DISTANCE aftr LEFT TURN IS %f\n", temp);
                if (wb_distance_sensor_get_value(ds[0])<750) {
                    printf("**************BOX_DETECTED*************");
                    state++;
                    line_follow = 1;
                }
                else {
                    state--;
                    printf("NO BOX ON RADIUS");
                    /*for (int i = 0; i < 130; i++) {
                        wb_motor_set_position(wheels[0], INFINITY);
                        wb_motor_set_velocity(wheels[0], -1);
                        wb_motor_set_position(wheels[1], INFINITY);
                        wb_motor_set_velocity(wheels[1], -1);
                        wb_robot_step(TIME_STEP);
                    }
                    */
                    
                    hardRightf(-1.25,-0.5,2.5);
                    quardrant++;
                    line_follow = 1;
                }
            }

           if (state == 6) {
                
                if (wb_distance_sensor_get_value(ds[0]) < 100) {
                    for (int i = 0; i < 25; i++) {
                        wb_motor_set_position(wheels[0], INFINITY);
                        wb_motor_set_velocity(wheels[0], 0.00628 * Speeds[0]);
                        wb_motor_set_position(wheels[1], INFINITY);
                        wb_motor_set_velocity(wheels[1], 0.00628 * Speeds[1]);
                        wb_robot_step(TIME_STEP);
                    }
                    printf("APRROACHED BOX at 10cm");
                    line_follow = 0;
                    line_follow = 0;

                    wb_motor_set_position(wheels[0], INFINITY);
                    wb_motor_set_velocity(wheels[0], 0);
                    wb_motor_set_position(wheels[1], INFINITY);
                    wb_motor_set_velocity(wheels[1], 0);
                    state++;
                }
                
            }
           if (state == 7) {
               printf("LIFTING BOX\n");
               Lifting_box(&linear, &S1, &S2, lr_1, lr_2, Sr_1, Sr_2);
               printf("BOX LIFTED, READING COLOR\n");
               unsigned short frontColor = readColor(CAM2);
               printf("Front Color \t%d\n", frontColor);
               unsigned short bottomColor = readColor(CAM1);
               printf("Bottom Color \t%d\n", bottomColor);
               if ((abs(frontColor -bottomColor)%2)==1) {
                   path = 'R';
               }
               else {
                   path = 'L';
               }
               printf("Path is%c", path);
               for (int i = 0; i < 100; i++) {
                   wb_robot_step(TIME_STEP);
               }
               printf("PLACING BOX BACK");
               Placing_box(&linear, &S1, lr_1, lr_2, Sr_1);
               printf("BACKING");
               for (int i = 0; i < 50; i++) {
                   wb_motor_set_position(wheels[0], INFINITY);
                   wb_motor_set_velocity(wheels[0], -1);
                   wb_motor_set_position(wheels[1], INFINITY);
                   wb_motor_set_velocity(wheels[1], -1);
                   wb_robot_step(TIME_STEP);
               }
               hardLeftf(2.7,2.5,-2.5);
               state++;
           }

           if (state ==8){
               line_follow = 1;
               if (quardrant < 4) {
                   if (junc == 3) {
                       hardLeftf(1.4, 2.5, -0.5);
                   }
                   if (junc == 2) {
                       hardRightf(-1.27, -0.5, 2.5);
                   }

               }
               else {

                   if (junc == 3) {
                       hardRightf(-1.4, -0.5, 2.5);
                   }
                   if (junc == 1) {
                       hardLeftf(1.4, 2.5, -0.5);
                   }

               }
               
           }
            
       
        wall_follow();
        ReadQTR2(QTR);
        lineFollow2(1);

       
        //printf("%4f   %4f   %4f   %4f    %4f    %4f   %4f    %4f", qtrNew[0], qtrNew[1], qtrNew[2], qtrNew[3], qtrNew[4], qtrNew[5], qtrNew[6], qtrNew[7]);
        printf("%4f   %4f   %4f   %4f    %4f    %4f   %4f    %4f", error[0], error[1], error[2], error[3], error[4], error[5], error[6], error[7]);
        /* Process sensor data here */
       
       // printf("\t CAM1  %c \t CAM2  %c\n",readColor(CAM1),readColor(CAM2));
        printf("STATE is %d \t LINE_FOLLOW is %d\n", state, line_follow);
        printf("\t \t \t FRONT IR %f", wb_distance_sensor_get_value(ds[0]));
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
