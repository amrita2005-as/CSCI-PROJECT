#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

int main(int argc, char **argv) {
    wb_robot_init();
    
    // Motor initialization
    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    
    double left_speed = MAX_SPEED;
    double right_speed = MAX_SPEED;

    // Distance sensors initialization
    WbDeviceTag distance_sensors[8];
    char dist_sensor_name[50];
    for (int i = 0; i < 8; i++) {
        sprintf(dist_sensor_name, "ps%d", i);
        distance_sensors[i] = wb_robot_get_device(dist_sensor_name);
        wb_distance_sensor_enable(distance_sensors[i], TIME_STEP);
    }
    
    while (wb_robot_step(TIME_STEP) != -1) {
        // Read distance sensor values
        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);
    }
    
    wb_robot_cleanup();
    return 0;
}
