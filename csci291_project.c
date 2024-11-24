/*
 *****************************************************************************
 *                              WEBOT MAZE PROJECT                           *
 * File: csci291_project.c                                                   *
 * Description: The e-puck moves along the left side of the maze wall.       *                                                                        *
 * Team members: Jia and Tendai                                              *                             *
 *                                                                           *                                                                         *
 *****************************************************************************                                                                             *
*/

#include <stdio.h>
#include <stdbool.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/light_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <math.h>

#define TIME_STEP 64 // Time step for Webots simulation
#define MAX_SPEED 6.28 // Maximum motor speed
#define WALL_THRESHOLD 100.0 // Threshold for wall detection
#define LIGHT_THRESHOLD 500.0 // Threshold for light intensity
#define GPS_THRESHOLD 0.07 // Threshold for GPS position comparison
#define MAX_DEAD_ENDS 10 // Maximum number of dead ends to store

// Struct to store information about dead ends
typedef struct {
    double light_intensity; // Light intensity at the dead end
    double coordinates[3]; // GPS coordinates of the dead end
} DeadEnd;

// Function prototypes
void init_devices(WbDeviceTag *left_motor, WbDeviceTag *right_motor, WbDeviceTag distance_sensors[], WbDeviceTag *light_sensor, WbDeviceTag *gps);
bool is_dead_end(WbDeviceTag distance_sensors[], double *dead_end_timer, int *dead_end_count);
void handle_dead_end(DeadEnd dead_ends[], int *nest, double light_value, const double gps_values[]);
void follow_wall(WbDeviceTag distance_sensors[], double *left_speed, double *right_speed);
bool target_reached(const double gps_values[], const double target_coordinates[]);
double clamp(double value, double min_value, double max_value);

int main() {
    wb_robot_init(); // Initialize Webots robot

    // Declare variables for devices and robot state
    WbDeviceTag left_motor, right_motor, distance_sensors[8], light_sensor, gps;
    init_devices(&left_motor, &right_motor, distance_sensors, &light_sensor, &gps);

    DeadEnd dead_ends[MAX_DEAD_ENDS]; // Array to store dead end information
    int nest = 0; // Number of recorded dead ends
    double dead_end_timer = 0; // Timer to manage dead end detection
    int dead_end_count = 0; // Counter for consecutive dead end detections
    double left_speed = MAX_SPEED, right_speed = MAX_SPEED; // Motor speeds
    bool target_reached_flag = false; // Flag to indicate target has been reached

    while (wb_robot_step(TIME_STEP) != -1) { // Main simulation loop
        const double *gps_values = wb_gps_get_values(gps); // Get GPS coordinates
        double light_value = wb_light_sensor_get_value(light_sensor); // Get light intensity

        // Check if the robot encounters a dead end
        if (is_dead_end(distance_sensors, &dead_end_timer, &dead_end_count)) {
            handle_dead_end(dead_ends, &nest, light_value, gps_values); // Handle the dead end

            // If all dead ends are recorded, identify the brightest dead end
            if (nest == MAX_DEAD_ENDS) {
                int brightest_index = 0;
                for (int i = 1; i < MAX_DEAD_ENDS; i++) {
                    if (dead_ends[i].light_intensity > dead_ends[brightest_index].light_intensity) {
                        brightest_index = i;
                    }
                }
                printf("\nBrightest light intensity: %f found at dead end %d\n", dead_ends[brightest_index].light_intensity, brightest_index + 1);
                printf("Position: x = %f, y = %f, z = %f\n", dead_ends[brightest_index].coordinates[0], dead_ends[brightest_index].coordinates[1], dead_ends[brightest_index].coordinates[2]);

                // Check if the robot has reached the target position
                if (target_reached(gps_values, dead_ends[brightest_index].coordinates)) {
                    printf("\nRobot at position with brightest light intensity reached!\n");
                    target_reached_flag = true;
                }
            }
        } else {
            follow_wall(distance_sensors, &left_speed, &right_speed); // Continue following the wall
        }

        // Set motor velocities with clamped values
        wb_motor_set_velocity(left_motor, clamp(left_speed, -MAX_SPEED, MAX_SPEED));
        wb_motor_set_velocity(right_motor, clamp(right_speed, -MAX_SPEED, MAX_SPEED));

        // Stop the robot if the target is reached
        if (target_reached_flag) {
            wb_motor_set_velocity(left_motor, 0);
            wb_motor_set_velocity(right_motor, 0);
            break;
        }
    }

    wb_robot_cleanup(); // Cleanup and terminate simulation
    return 0;
}

// Initialize robot devices
void init_devices(WbDeviceTag *left_motor, WbDeviceTag *right_motor, WbDeviceTag distance_sensors[], WbDeviceTag *light_sensor, WbDeviceTag *gps) {
    *left_motor = wb_robot_get_device("left wheel motor"); // Get left motor device
    *right_motor = wb_robot_get_device("right wheel motor"); // Get right motor device
    wb_motor_set_position(*left_motor, INFINITY); // Set motor to velocity control mode
    wb_motor_set_position(*right_motor, INFINITY);

    // Initialize distance sensors
    for (int i = 0; i < 8; i++) {
        char dist_sensor_name[50];
        sprintf(dist_sensor_name, "ps%d", i); // Generate sensor name
        distance_sensors[i] = wb_robot_get_device(dist_sensor_name); // Get sensor device
        wb_distance_sensor_enable(distance_sensors[i], TIME_STEP); // Enable sensor
    }

    *light_sensor = wb_robot_get_device("ls0"); // Get light sensor device
    wb_light_sensor_enable(*light_sensor, TIME_STEP); // Enable light sensor

    *gps = wb_robot_get_device("gps"); // Get GPS device
    wb_gps_enable(*gps, TIME_STEP); // Enable GPS
}

// Detect if the robot encounters a dead end
bool is_dead_end(WbDeviceTag distance_sensors[], double *dead_end_timer, int *dead_end_count) {
    double front_distance = wb_distance_sensor_get_value(distance_sensors[0]); // Get front sensor value
    double current_time = wb_robot_get_time(); // Get current simulation time

    // Check if the front is blocked and manage dead end detection timing
    if (front_distance > 100 && (*dead_end_count == 0 || (current_time - *dead_end_timer) > 1.7)) {
        *dead_end_timer = current_time;
        (*dead_end_count)++;
    }

    // Confirm dead end after two consecutive detections
    if (*dead_end_count >= 2) {
        *dead_end_count = 0;
        return true;
    }

    // Reset dead end count if no detection for a long time
    if ((current_time - *dead_end_timer) > 10.0) {
        *dead_end_count = 0;
    }

    return false;
}

// Handle dead end information storage
void handle_dead_end(DeadEnd dead_ends[], int *nest, double light_value, const double gps_values[]) {
    if (*nest < MAX_DEAD_ENDS) { // Check if storage limit is not exceeded
        dead_ends[*nest].light_intensity = light_value; // Store light intensity
        for (int i = 0; i < 3; i++) {
            dead_ends[*nest].coordinates[i] = gps_values[i]; // Store GPS coordinates
        }
        printf("\nDead end %d. Light intensity: %f\n", *nest + 1, light_value);
        printf("Coordinates: x = %f, y = %f, z = %f\n", gps_values[0], gps_values[1], gps_values[2]);
        (*nest)++; // Increment dead end counter
    }
}

// Navigate the robot along walls
void follow_wall(WbDeviceTag distance_sensors[], double *left_speed, double *right_speed) {
    bool left_wall = wb_distance_sensor_get_value(distance_sensors[5]) > WALL_THRESHOLD; // Check left wall
    bool left_corner = wb_distance_sensor_get_value(distance_sensors[6]) > WALL_THRESHOLD; // Check left corner
    bool front_wall = wb_distance_sensor_get_value(distance_sensors[7]) > WALL_THRESHOLD; // Check front wall

    // Adjust motor speeds based on wall positions
    if (front_wall) {
        *left_speed = MAX_SPEED;
        *right_speed = -MAX_SPEED; // Turn to avoid collision
    } else if (left_wall) {
        *left_speed = MAX_SPEED / 2;
        *right_speed = MAX_SPEED / 2; // Move along the wall
    } else if (left_corner) {
        *left_speed = MAX_SPEED;
        *right_speed = MAX_SPEED / 4; // Turn slightly to
    } else {
        *left_speed = MAX_SPEED / 4;
        *right_speed = MAX_SPEED;
    }
}

bool target_reached(const double gps_values[], const double target_coordinates[]) {
    return fabs(gps_values[0] - target_coordinates[0]) < GPS_THRESHOLD &&
           fabs(gps_values[1] - target_coordinates[1]) < GPS_THRESHOLD &&
           fabs(gps_values[2] - target_coordinates[2]) < GPS_THRESHOLD;
}

double clamp(double value, double min_value, double max_value) {
    return fmin(fmax(value, min_value), max_value);
}

