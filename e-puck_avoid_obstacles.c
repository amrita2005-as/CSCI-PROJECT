#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/gps.h>
#include <math.h>

#define TIME_STEP 64       // Time step for the Webots simulation
#define MAX_SPEED 6.28     // Maximum speed of the robot
#define LIGHT_THRESHOLD 500.0 // Threshold for light detection
#define WALL_THRESHOLD 100.0  // Threshold for wall detection
#define GPS_THRESHOLD 0.07    // Threshold for proximity detection using GPS

// Function to regulate motor speed within a defined range
double regulate_speed(double value, double min_value, double max_value) {
    if (value < min_value) {
        return min_value;
    } else if (value > max_value) {
        return max_value;
    }
    return value;
}

// Main function
int main(int argc, char **argv) {
  // Initialize the Webots API
  wb_robot_init();
  
  // Define and enable the motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY); // Set position to infinity for velocity control
  wb_motor_set_position(right_motor, INFINITY);
  
  // Initialize motor speeds
  double light_deadend[11];         // Array to store light intensities at dead-ends
  double gps_coordinates[10][3];    // Array to store GPS coordinates of dead-ends
  double left_speed = MAX_SPEED;    // Initial speed of the left motor
  double right_speed = MAX_SPEED;   // Initial speed of the right motor
  int nest = 0;                     // Counter for detected dead-ends
  double max = 0;                   // Variable to track maximum light intensity
  int j = 0;                        // Index of the brightest dead-end
  
  printf("\n-----------------------------------------------------\n");
  printf("Welcome to Webots Robot Maze Simulator\n\n");
  
  // Define and enable the distance sensors
  WbDeviceTag distance_sensors[8];
  char dist_sensor_name[50];
  for (int i = 0; i < 8; i++) {
    sprintf(dist_sensor_name, "ps%d", i); // Create sensor name dynamically
    distance_sensors[i] = wb_robot_get_device(dist_sensor_name);
    wb_distance_sensor_enable(distance_sensors[i], TIME_STEP);
  }
  
  // Define and enable the light sensor
  WbDeviceTag light_sensor = wb_robot_get_device("ls0"); 
  wb_light_sensor_enable(light_sensor, TIME_STEP);

  // Enable the GPS sensor
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  // Variables for dead-end detection
  int dead_end_count = 0;       // Counter for consecutive dead-end detections
  double dead_end_timer = 0;    // Timer to track detection intervals

  // Flag to indicate if the target has been reached
  bool target_reached = false;

  // Function to check if the robot is in a dead-end
  bool is_dead_end() {
    double front_distance = wb_distance_sensor_get_value(distance_sensors[0]); // Check front sensor
    double current_time = wb_robot_get_time();

    // If a wall is detected in front and it's within the detection interval
    if (front_distance > 100) {
      if (dead_end_count == 0 || (current_time - dead_end_timer) > 1.7) {
        dead_end_timer = current_time; // Reset the timer
        dead_end_count++;
      }
    }

    // If two consecutive detections occur within 4 seconds, mark as a dead-end
    if (dead_end_count >= 2) {
      dead_end_count = 0; // Reset count
      return true;
    }

    // Reset the counter if no detection occurs for 10 seconds
    if ((current_time - dead_end_timer) > 10.0) {
      dead_end_count = 0;
    }
    
    return false;
  }
  
  // Main control loop
  while (wb_robot_step(TIME_STEP) != -1) {
    // Get the current GPS coordinates
    const double *gps_values = wb_gps_get_values(gps);

    // Read distance sensor values for wall-following
    bool left_wall = wb_distance_sensor_get_value(distance_sensors[5]) > WALL_THRESHOLD; // Left side
    bool left_corner = wb_distance_sensor_get_value(distance_sensors[6]) > WALL_THRESHOLD; // Left diagonal
    bool front_wall = wb_distance_sensor_get_value(distance_sensors[7]) > WALL_THRESHOLD; // Front

    // Read light sensor value
    double light_value = wb_light_sensor_get_value(light_sensor);

    // Check for a dead-end
    if (is_dead_end()) {
      if (nest < 10) {
        // Store light and GPS values for the dead-end
        nest++;
        light_deadend[nest] = light_value;
        gps_coordinates[nest - 1][0] = gps_values[0]; // x-coordinate
        gps_coordinates[nest - 1][1] = gps_values[1]; // y-coordinate
        gps_coordinates[nest - 1][2] = gps_values[2]; // z-coordinate

        // Log dead-end details
        printf("Dead-end %d reached.\n", nest);
        printf("Light intensity recorded: %f\n", light_deadend[nest]);
        printf("Coordinates logged: x = %f, y = %f, z = %f\n", gps_values[0], gps_values[1], gps_values[2]);
      }

      if (nest == 10) {
        // Find the dead-end with the highest light intensity
        for (int i = 1; i <= 10; i++) {
          if (light_deadend[i] > max) {
            max = light_deadend[i];
            j = i;
          }
        }
        printf("Highest light intensity observed: %f at dead-end %d\n", max, j);
        printf("Brightest location: x = %f, y = %f, z = %f\n",
               gps_coordinates[j - 1][0], gps_coordinates[j - 1][1], gps_coordinates[j - 1][2]);
      }

      // Check if the target location has been reached
      if (fabs(gps_values[0] - gps_coordinates[j - 1][0]) < GPS_THRESHOLD &&
          fabs(gps_values[1] - gps_coordinates[j - 1][1]) < GPS_THRESHOLD &&
          fabs(gps_values[2] - gps_coordinates[j - 1][2]) < GPS_THRESHOLD) {
          printf("\n\nTarget dead-end with maximum light intensity successfully reached!\n\n");
          target_reached = true;
      }
    } else {
      // Wall-following logic
      if (front_wall) {
        left_speed = MAX_SPEED; // Turn right
        right_speed = -MAX_SPEED;
      } else if (left_wall) {
        left_speed = MAX_SPEED / 2; // Go straight
        right_speed = MAX_SPEED / 2;
      } else if (left_corner) {
        left_speed = MAX_SPEED; // Slight left turn
        right_speed = MAX_SPEED / 4;
      } else {
        left_speed = MAX_SPEED / 4; // Turn left
        right_speed = MAX_SPEED;
      }
    }

    // Regulate speeds
    left_speed = regulate_speed(left_speed, -MAX_SPEED, MAX_SPEED);
    right_speed = regulate_speed(right_speed, -MAX_SPEED, MAX_SPEED);

    // Set motor velocities
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);

    // Stop if the target is reached
    if (target_reached) {
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
      break;
    }
  }

  // Clean up Webots API
  wb_robot_cleanup();

  return 0;
}
