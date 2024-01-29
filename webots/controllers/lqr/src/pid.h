#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct{
    double k_vertical_thrust;       // minimum thrust required for drone takeoff.
    double k_vertical_offset;       // vertical displacement of gravity center of quadcopter from ground after landing
    bool led_state;                 // state of on-board LEDs
    double roll_disturbance;        // perturbation of roll angle by keyboard input
    double pitch_disturbance;       // perturbation of pitch angle by keyboard input
    double yaw_disturbance;         // perturbation of yaw angle by keyboard input
    double control_roll;            // control input of roll angle of quadcopter
    double control_pitch;           // control input of pitch angle of quadcopter
    double control_yaw;             // control input of yaw angle of quadcopter
    double control_altitude;        // control input of flight altitude of a quadcopter
    double altitude_difference;     // difference of altitude of drone from desired altitude
    double front_left_motor_input;  // input value for front left motor of the quadcopter
    double front_right_motor_input; // input value for front right motor of the quadcopter
    double back_left_motor_input;   // input value for back left motor of the quadcopter
    double back_right_motor_input;  // input value for back right motor of the quadcopter
} _controller;

typedef struct{
    double kp_altitude;             // proportional gain of the altitude PID controller
    double kp_roll;                 // proportional gain of the roll angle PID controller
    double kp_pitch;                // proportional gain of the pitch angle PID controller
    double desired_altitude;        // desired altitude, as specified by the user
} _pid;

void PID_init();

double PID_realize(int i);

#endif // ARCHIVE_H
