#ifndef SAVEFILE_H
#define SAVEFILE_H

#define ARRAY_SIZE 1000       // sampling times

typedef struct{
    double position_x[ARRAY_SIZE];              // x-axis position of drone
    double position_y[ARRAY_SIZE];              // y-axis position of drone
    double position_z[ARRAY_SIZE];              // z-axis position of drone
    double velocity_x[ARRAY_SIZE];              // x-axis velocity of drone
    double velocity_y[ARRAY_SIZE];              // y-axis velocity of drone
    double velocity_z[ARRAY_SIZE];              // z-axis velocity of drone
    double roll[ARRAY_SIZE];                    // roll angle of drone
    double pitch[ARRAY_SIZE];                   // pitch angle of drone
    double yaw[ARRAY_SIZE];                     // yaw angle of drone
    double velocity_i[ARRAY_SIZE];              // i-axis velocity of drone represented in body frame
    double velocity_j[ARRAY_SIZE];              // j-axis velocity of drone represented in body frame
    double velocity_k[ARRAY_SIZE];              // k-axis velocity of drone represented in body frame
    double acceleration_x[ARRAY_SIZE];          // x-axis acceleration of drone
    double acceleration_y[ARRAY_SIZE];          // y-axis acceleration of drone
    double acceleration_z[ARRAY_SIZE];          // z-axis acceleration of drone
    double roll_angular_velocity[ARRAY_SIZE];   // roll angular velocity of drone
    double pitch_angular_velocity[ARRAY_SIZE];  // pitch angular velocity of drone
    double yaw_angular_velocity[ARRAY_SIZE];    // yaw angular velocity of drone
    double angular_velocity_x[ARRAY_SIZE];      // x-axis angular velocity of drone
    double angular_velocity_y[ARRAY_SIZE];      // y-axis angular velocity of drone
    double angular_velocity_z[ARRAY_SIZE];      // z-axis angular velocity of drone
    double control_roll[ARRAY_SIZE];            // control input of roll angle of quadcopter
    double control_pitch[ARRAY_SIZE];           // control input of pitch angle of quadcopter
    double control_yaw[ARRAY_SIZE];             // control input of yaw angle of quadcopter
    double control_altitude[ARRAY_SIZE];        // control input of flight altitude of a quadcopter
    double altitude_difference[ARRAY_SIZE];     // difference of altitude of drone from desired altitude
    double front_left_motor_input[ARRAY_SIZE];  // input value for front left motor of the quadcopter
    double front_right_motor_input[ARRAY_SIZE]; // input value for front right motor of the quadcopter
    double back_left_motor_input[ARRAY_SIZE];   // input value for back left motor of the quadcopter
    double back_right_motor_input[ARRAY_SIZE];  // input value for back right motor of the quadcopter
} _archive;

void saveArchiveToTxt(double *archive1, int size, const char *filename);

void saveArchive();

#endif // ARCHIVE_H
