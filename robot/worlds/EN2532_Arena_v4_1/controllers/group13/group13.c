/*
 * File:          group13.c
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
double qtrStore[8];
char QTR_names[nQTR][6] = { "qtr0","qtr1","qtr2","qtr3","qtr4","qtr5","qtr6","qtr7" };
char wheels_names[2][12] = { "right_motor", "left_motor" };
unsigned int error[8] = { 0,0,0,0,0,0,0,0 };
double weights[8] = { 0,1000,2000,3000,4000,5000,6000,7000};

//
double Kp = 0.5;
double Kd = 0.3;                                                          
double P = 0;
double D = 0;
double pEr = 0;
double baseSpeed = 750;  //  rad/s
double Speeds[2] = { 0,0 };
double PID = 0;
short junc = -1;
double eSUM = 0;
double coeff = 0.25;
unsigned short int state = 3;
char ds_names[3][10] = { "front_ir","left_ir","right_ir" };
unsigned short quadrant = 0;
unsigned char path ;// = 'L'
unsigned int left = 0;
unsigned int right = 0;
int line_follow = 1;
unsigned int LINE_THRESH =  500;
int wall_flag = 0;
unsigned char ramp='0';
short int preVjunc = 0;
unsigned short int tcount = 0;
char qArray[4] = { '1','4','3','2'};
int lCount = 0;
int tempCount = 0;

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



void ReadQTR2(WbDeviceTag QTRa[]) {

        qtrStore[0] = wb_distance_sensor_get_value(QTRa[0]);
        qtrStore[1] = wb_distance_sensor_get_value(QTRa[1]);
        qtrStore[2] = wb_distance_sensor_get_value(QTRa[2]);
        qtrStore[3] = wb_distance_sensor_get_value(QTRa[3]);
        qtrStore[4] = wb_distance_sensor_get_value(QTRa[4]);
        qtrStore[5] = wb_distance_sensor_get_value(QTRa[5]);
        qtrStore[6] = wb_distance_sensor_get_value(QTRa[6]);
        qtrStore[7] = wb_distance_sensor_get_value(QTRa[7]);

        error[0] = ((LINE_THRESH - qtrStore[0]) > 0);
        error[1] = ((LINE_THRESH - qtrStore[1]) > 0);
        error[2] = ((LINE_THRESH - qtrStore[2]) > 0);
        error[3] = ((LINE_THRESH - qtrStore[3]) > 0);
        error[4] = ((LINE_THRESH - qtrStore[4]) > 0);
        error[5] = ((LINE_THRESH - qtrStore[5]) > 0);
        error[6] = ((LINE_THRESH - qtrStore[6]) > 0);
        error[7] = ((LINE_THRESH - qtrStore[7]) > 0);
}

short j_check(void) {

    ReadQTR2(QTR);
    if ((error[0] ==1) && (error[1] == 1) && (error[2] == 1) && (error[3] == 1) && (error[4] == 1) && (error[5] == 0) && (error[6] == 0)&&(error[7] == 0)) {
        printf("\t left junction found\t");
        return 1;

    }
    else if ((error[0] == 1) && (error[1] == 1) && (error[2] == 1) && (error[3] == 1) && (error[4] == 1) && (error[5] == 1) && (error[6] == 0) && (error[7] == 0)) {
        printf("\tleft junction found\t");
        return 1;

    }

    else if ((error[0] == 0) && (error[1] == 0) && (error[2] == 1) && (error[3] == 1) && (error[4] == 1) && (error[5] == 1) && (error[6] == 1)&&(error[7]==1)) {

        printf("\tright junction found\t");
        return 2;
    }
    else if ((error[0] == 0) && (error[1] == 1) && (error[2] == 1) && (error[3] == 1) && (error[4] == 1) && (error[5] == 1) && (error[6] == 1) && (error[7] == 1)) {

        printf("\tright junction found\t");
        return 2;
    }

    else if ((error[0] == 1) && (error[1] == 1) && (error[2] == 1) && (error[3] == 1) && (error[4] == 1) && (error[5] == 1) && (error[6] == 1) && (error[7] == 1)) {
        printf("\tT junction found \t");
        return 3;

    }
     /*unsigned int flag = 1;
     for (int i = 0; i < 8; i++) {
         flag = flag && (error[i] == 1);
         wb_robot_step(TIME_STEP);
     }
     if(flag) {
         return 3;
     }
     */

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
    printf("InitAngle = %f\n", initAngle);
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
void hardRightf(double angle, double RightSpeed, double LeftSpeed) {
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


void lineFollow2(double coeff) {
    if (line_follow == 1) {
        eSUM = 0;
        double denominator = 0;
        for (int i = 0; i < 8; i++) { 
            eSUM += (error[i] * weights[i]);
            denominator += error[i];
        }
        printf("RAW eSUM = %f\n", eSUM);
        printf("Denominator = %f\n", denominator);
        if (denominator != 0) {
            eSUM = eSUM / denominator;
        }
        
        eSUM -= 3500;

        if (state == 9) {
            if (denominator == 0) {
                eSUM = 0;
            }
        }
        
        printf("%f\n", eSUM);
        P = Kp * eSUM;
        D = Kd * (eSUM - pEr);
        PID = P + D;
        Speeds[0] = (baseSpeed / coeff) - PID;
        Speeds[1] = (baseSpeed / coeff) + PID;
        pEr = eSUM;
        

        wb_motor_set_position(wheels[0], INFINITY);
        wb_motor_set_velocity(wheels[0], 0.002 * Speeds[0]);
        wb_motor_set_position(wheels[1], INFINITY);
        wb_motor_set_velocity(wheels[1], 0.002 * Speeds[1]);
    }
}

unsigned short int readColor(WbDeviceTag camera) {
    /*
    Returns the color detected as a char 'R','G','B'
    the color test is performed in the given order
    assumes the object is ONLY one of the three colors
    for example white will be detected as R in here.


    Input should be an initialized WbDeviceTag device


    */
    unsigned char* image = wb_camera_get_image(camera);
    unsigned short levels[3];
    levels[0] = wb_camera_image_get_red(image, 8, 4, 4);
    levels[1] = wb_camera_image_get_green(image, 8, 4, 4);
    levels[2] = wb_camera_image_get_blue(image, 8, 4, 4);
    unsigned int max = 0;
    unsigned short int maxIndex = 0;
    for (unsigned short int i = 0; i < 3; i++) {
        if (levels[i] > max) {
            max = levels[i];
            maxIndex = i;
        }
    }
    return maxIndex + 1;

    /*
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
    */
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

    while (*linear > -0.03) {
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

    while (*S1 >= 0.3) {
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

    for (int i = 0; i < 3; i++) {

        ds[i] = wb_robot_get_device(ds_names[i]);
        wb_distance_sensor_enable(ds[i], TIME_STEP);
        printf("Initialized ds %d\n", i);
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
        printf("\t Junc is %d\n", junc);
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


        if ((state == 1) || (state == 3)) {
                if (junc == 1) {
                    for (int i = 0; i < 110; i++) {
                        wb_motor_set_position(wheels[0], INFINITY);
                        wb_motor_set_velocity(wheels[0], 1.5);
                        wb_motor_set_position(wheels[1], INFINITY);
                        wb_motor_set_velocity(wheels[1], 1.5);
                        wb_robot_step(TIME_STEP);
                    }
                    hardLeftf(1.55, 1, -1);
                    coeff = 0.5;
                }
                if (junc == 2) {
                    for (int i = 0; i < 110; i++) {
                        wb_motor_set_position(wheels[0], INFINITY);
                        wb_motor_set_velocity(wheels[0], 1.5);
                        wb_motor_set_position(wheels[1], INFINITY);
                        wb_motor_set_velocity(wheels[1], 1.5);
                        wb_robot_step(TIME_STEP);
                    }
                    hardRightf(-1.55, -1, 1);
                }
        }
        
        if ((state == 2) && (wb_distance_sensor_get_value(ds[2]) >= 1000) && (wb_distance_sensor_get_value(ds[1]) >= 1000)) {
            for (int i = 0; i < 20; i++) {
                wb_motor_set_position(wheels[0], INFINITY);
                wb_motor_set_velocity(wheels[0], 1);
                wb_motor_set_position(wheels[1], INFINITY);
                wb_motor_set_velocity(wheels[1], 1);
                wb_robot_step(TIME_STEP);
            }
            line_follow = 1;
            wall_flag = 0;
            coeff = 0.25;
            state++;
        }
        /*(state == 1) ||*/
       /* if(state == 3) {

            if (junc == 1) {
                hardLeftf(1.25, 2.5, -0.5);
            }
            if (junc == 2) {
                hardRightf(-1.47, -0.5, 2.5);
                junc = -1;
            }

        }*/

        if ((state == 3) && (junc == 3)) {
            for (int i = 0; i < 110; i++) {
                wb_motor_set_position(wheels[0], INFINITY);
                wb_motor_set_velocity(wheels[0], 1.5);
                wb_motor_set_position(wheels[1], INFINITY);
                wb_motor_set_velocity(wheels[1], 1.5);
                wb_robot_step(TIME_STEP);
            }
            hardRightf(-1.3, -1, 1);
            line_follow = 1;
            Kp = 0.1;
            quadrant = 1;
            state++;

        }
        if (state == 4) {
            coeff = 0.5;
            Kp = 0.65;
            Kd = 0.4;
            /*if ((junc == 1) && (preVjunc - 1)) {
                    lCount++;
                }
                */
            if (junc == 1) {
                line_follow = 0;
                for (int i = 0; i < 120; i++) {
                    wb_motor_set_position(wheels[0], INFINITY);
                    wb_motor_set_velocity(wheels[0], 1.5);
                    wb_motor_set_position(wheels[1], INFINITY);
                    wb_motor_set_velocity(wheels[1], 1.2);
                    wb_robot_step(TIME_STEP);
                }
                hardLeftf(1.57, 1, -1);
                state++;
                printf("STATE is %d \t LINE_FOLLOW is %d\n", state, line_follow);
            }
            preVjunc = junc;
        }
        if (state == 5) {
           /* for (int i = 0; i < 100; i++) {
                line_follow = 1;
                lineFollow2(2);
                wb_robot_step(TIME_STEP);
            }
            */
            line_follow = 0;
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
            if (wb_distance_sensor_get_value(ds[0]) < 750) {
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

                hardRightf(-1.25, -0.5, 2.5);
                quadrant++;
                line_follow = 1;
                lCount++;
            }
        }

        if (state == 6) {
            if (wb_distance_sensor_get_value(ds[0]) < 100) {
                line_follow = 0;
                for (int i = 0; i < 100; i++) {
                    wb_motor_set_velocity(wheels[0], 0);
                    wb_motor_set_velocity(wheels[1], 0);
                    wb_robot_step(TIME_STEP);
                }
                printf("right speed%f\n", Speeds[0]);
                printf("right speed%f\n", Speeds[1]);
                for (int i = 0; i < 55; i++) {
                    wb_motor_set_position(wheels[0], INFINITY);
                    wb_motor_set_velocity(wheels[0], 2);
                    wb_motor_set_position(wheels[1], INFINITY);
                    wb_motor_set_velocity(wheels[1], 2);
                    wb_robot_step(TIME_STEP);
                }
                printf("APRROACHED BOX at 10cm");
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
            for (int i = 0; i < 30; i++) {
                wb_robot_step(TIME_STEP);
            }
            unsigned short int frontColor = readColor(CAM1);
            printf("Front Color \t%d\n", frontColor);
            unsigned short int bottomColor = readColor(CAM2);
            printf("Bottom Color \t%d\n", bottomColor);
            if ((abs(frontColor - bottomColor) % 2) == 1) {
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
            hardLeftf(2.8, 2.5, -2.5);
            state++;
        }

        if (state == 8) {
            if (junc == 1) {
                tempCount++;
                printf("\ttempCount %d\t\n", tempCount);
                if (tempCount > 15) {
                    lCount++;
                    tempCount = 0;
                }
            }
            preVjunc = junc;
            line_follow = 1;
            if (quadrant < 4) {
                if (junc == 3) {
                    hardLeftf(1.65, 2.5, -0.5);
                    lCount++;
                }
                if (junc == 2) {
                    hardRightf(-1.27, -0.5, 2.5);
                    state++;
                }

            }
            else {

                if (junc == 3) {
                    hardRightf(-1.4, -0.5, 2.5);
                }
                if (junc == 1) {
                    hardLeftf(1.4, 2.5, -0.5);
                    state++;
                }

            }

        }

        if (state == 9) {
            //Kp = 0.5;
            line_follow = 1;
            coeff = 1.5;
            if (wb_inertial_unit_get_roll_pitch_yaw(IMU)[1] > 0.1) {
                ramp = 'A';
            }
            if (junc == 3) {
                if (path == 'R') {
                    for (int i = 0; i < 100; i++) {
                        wb_motor_set_position(wheels[0], INFINITY);
                        wb_motor_set_velocity(wheels[0], 1.5);
                        wb_motor_set_position(wheels[1], INFINITY);
                        wb_motor_set_velocity(wheels[1], 1.5);
                        wb_robot_step(TIME_STEP);
                    }
                    hardRightf(-1.4, -1, 1);
                    //state++;
                }
                else if (path == 'L') {
                    for (int i = 0; i < 100; i++) {
                        wb_motor_set_position(wheels[0], INFINITY);
                        wb_motor_set_velocity(wheels[0], 1.5);
                        wb_motor_set_position(wheels[1], INFINITY);
                        wb_motor_set_velocity(wheels[1], 1.5);
                        wb_robot_step(TIME_STEP);
                    }
                    hardLeftf(1.4, 1, -1);
                    //state++;
                }
            }
            if (wb_inertial_unit_get_roll_pitch_yaw(IMU)[1] < -0.3) {
                ramp = 'D';
                state++;
            }
        }

        if (state == 10) {
            coeff = 1.5;
            Kp = 0.0214;
            Kd = 0.005;
            LINE_THRESH = 800;
            line_follow = 1;
            /*wb_motor_set_position(wheels[0], INFINITY);
            wb_motor_set_velocity(wheels[0], 0.1);
            wb_motor_set_position(wheels[1], INFINITY);
            wb_motor_set_velocity(wheels[1], 0.1);
            */

            if (wb_inertial_unit_get_roll_pitch_yaw(IMU)[1] > 0.005) {
                ramp = 'E';
                state++;
            }
        }

        if (state == 11) {
            line_follow = 1;
            coeff = 1;
            Kp = 0.65;
            Kd = 0.3;
            if (junc == 1) {
                for (int i = 0; i < 110; i++) {
                    wb_motor_set_position(wheels[0], INFINITY);
                    wb_motor_set_velocity(wheels[0], 1.5);
                    wb_motor_set_position(wheels[1], INFINITY);
                    wb_motor_set_velocity(wheels[1], 1.5);
                    wb_robot_step(TIME_STEP);
                }
                hardLeftf(1.5, 1, -1);
                state++;
            }
            if (junc == 2) {
                for (int i = 0; i < 110; i++) {
                    wb_motor_set_position(wheels[0], INFINITY);
                    wb_motor_set_velocity(wheels[0], 1.5);
                    wb_motor_set_position(wheels[1], INFINITY);
                    wb_motor_set_velocity(wheels[1], 1.5);
                    wb_robot_step(TIME_STEP);
                }
                hardRightf(-1.5, -1, 1);
                state++;
            }
        }
        

        if (state == 12) {
            if (junc == 3) {
                line_follow = 0;
                wb_motor_set_position(wheels[0], INFINITY);
                wb_motor_set_velocity(wheels[0], 0);
                wb_motor_set_position(wheels[1], INFINITY);
                wb_motor_set_velocity(wheels[1], 0);
                state++;
            }
        }
        if (state == 13) {
            if (wb_distance_sensor_get_value(ds[0]) < 300) {
                state++;
            }
        }

        if (state == 14) {
            if (wb_distance_sensor_get_value(ds[0]) > 300) {
                coeff = 0.5;
                line_follow = 1;
                state++;
                preVjunc = 0;
            }
        }

        if (state == 15) {
            
            if ((preVjunc == -1) && (junc == 3)) {
                printf("\n*********************************TRANSITION FOUND**************************************");
                tcount++;
            }
            if (tcount ==2) {
                state++;
            }
            preVjunc = junc;
        }

        if (state == 16) {
            if (junc == 3) {
                line_follow = 0;
                wb_motor_set_position(wheels[0], INFINITY);
                wb_motor_set_velocity(wheels[0], 0);
                wb_motor_set_position(wheels[1], INFINITY);
                wb_motor_set_velocity(wheels[1], 0);
                state++;
            }  
        }

        if (state == 17) {
            for (int i = 0; i < 225; i++) {
                wb_motor_set_position(wheels[0], INFINITY);
                wb_motor_set_velocity(wheels[0], 1);
                wb_motor_set_position(wheels[1], INFINITY);
                wb_motor_set_velocity(wheels[1], 1);
                wb_robot_step(TIME_STEP);
                printf("FINAL LOOP\n");
            }
            wb_motor_set_position(wheels[0], INFINITY);
            wb_motor_set_velocity(wheels[0], 0);
            wb_motor_set_position(wheels[1], INFINITY);
            wb_motor_set_velocity(wheels[1], 0);
            state ++;
        }

        wall_follow();
        ReadQTR2(QTR);
        lineFollow2(coeff);


        printf("%4f   %4f   %4f   %4f    %4f    %4f   %4f    %4f\n", qtrStore[0], qtrStore[1], qtrStore[2], qtrStore[3], qtrStore[4], qtrStore[5], qtrStore[6], qtrStore[7]);
        printf("%d   %d   %d   %d    %d    %d   %d   %d\t\n", error[0], error[1], error[2], error[3], error[4], error[5], error[6],error[7]);
        /* Process sensor data here */

       // printf("\t CAM1  %c \t CAM2  %c\n",readColor(CAM1),readColor(CAM2));
        printf("    STATE is %d \t LINE_FOLLOW is %d", state, line_follow);
        printf("\t FRONT IR %f", wb_distance_sensor_get_value(ds[0]));
        printf("   Pitch value %f\t QUARDRANT = %c   QPOINTER = %d  ", wb_inertial_unit_get_roll_pitch_yaw(IMU)[1],qArray[lCount],lCount);
        printf("RAMP STATUS = %c\t TCOUNT %d\t LS %f RS %f\n",ramp,tcount,Speeds[0],Speeds[1]);
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
