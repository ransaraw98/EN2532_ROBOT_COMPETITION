#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 64

int main(int argc, char** argv) {
    wb_robot_init();
    int i;
    bool avoid_obstacle_counter = 0;
    WbDeviceTag ds[2];
    char ds_names[1][8] = {"front_ir"};
    for (i = 0; i < 1; i++) {
        ds[i] = wb_robot_get_device(ds_names[i]);
        wb_distance_sensor_enable(ds[i], TIME_STEP);
    }
    WbDeviceTag wheels[4];
    char wheels_names[4][12] = { "left_motor", "right_motor" };
    for (i = 0; i < 2; i++) {
        wheels[i] = wb_robot_get_device(wheels_names[i]);
        wb_motor_set_position(wheels[i], INFINITY);
    }
    while (wb_robot_step(TIME_STEP) != -1) {
        double left_speed = 1.0;
        double right_speed = 1.0;
        if (avoid_obstacle_counter > 0) {
            avoid_obstacle_counter--;
            left_speed = 1.0;
            right_speed = -1.0;
        }
        else { // read sensors
            double ds_values[2];
            for (i = 0; i < 2; i++)
                ds_values[i] = wb_distance_sensor_get_value(ds[i]);
            if (ds_values[0] < 950.0 || ds_values[1] < 950.0)
                avoid_obstacle_counter = 100;
        }
        wb_motor_set_velocity(wheels[0], left_speed);
        wb_motor_set_velocity(wheels[1], right_speed);

    }
    wb_robot_cleanup();
    return 0;  // EXIT_SUCCESS
}