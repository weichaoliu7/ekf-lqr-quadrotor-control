#include <stdio.h>
#include <stdbool.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>

#include "src.h"
#include "savefile.h"
#include "ekf.h"
#include "pid.h"

#define TIME_STEP 10

extern _archive archive;
extern _ekf ekf;

_controller controller;
_pid pid;

void PID_init(){
    wb_robot_init();

    // obtain four rotor motors and set them to speed mode
    WbDeviceTag front_right_motor = wb_robot_get_device("m1_motor");
    WbDeviceTag back_right_motor = wb_robot_get_device("m2_motor");
    WbDeviceTag back_left_motor = wb_robot_get_device("m3_motor");
    WbDeviceTag front_left_motor = wb_robot_get_device("m4_motor");
    WbDeviceTag motors[4] = {front_right_motor, back_right_motor, back_left_motor, front_left_motor};
    for (int m = 0; m < 4; ++m){
        wb_motor_set_position(motors[m], INFINITY);
        // wb_motor_set_velocity(motors[m], 1.0);
    }

    wb_motor_set_velocity(front_right_motor, -57);
    wb_motor_set_velocity(back_right_motor, 57);
    wb_motor_set_velocity(back_left_motor, -57);
    wb_motor_set_velocity(front_left_motor, 57);

    // display the welcome message
    printf("Start the drone...\n");

    // display manual control message
    printf("You can control the drone with your computer keyboard:\n");
    printf("- 'up': move forward.\n");
    printf("- 'down': move backward.\n");
    printf("- 'right': turn right.\n");
    printf("- 'left': turn left.\n");
    printf("- 'shift + up': increase the target altitude.\n");
    printf("- 'shift + down': decrease the target altitude.\n");
    printf("- 'shift + right': strafe right.\n");
    printf("- 'shift + left': strafe left.\n");

    controller.k_vertical_thrust = 56.5; // minimum thrust required for drone takeoff.
    controller.k_vertical_offset = 0.6;  // vertical displacement of gravity center of quadcopter from ground after landing
    pid.kp_altitude = 3.0;        // proportional gain of the altitude PID controller
    pid.kp_roll = 12.5;            // proportional gain of the roll angle PID controller
    pid.kp_pitch = 7.5;           // proportional gain of the pitch angle PID controller
    pid.desired_altitude = 1.0;     // desired altitude, as specified by the user
}

double PID_realize(int i){

    // obtain four rotor motors and set them to speed mode
    WbDeviceTag front_right_motor = wb_robot_get_device("m1_motor");
    WbDeviceTag back_right_motor = wb_robot_get_device("m2_motor");
    WbDeviceTag back_left_motor = wb_robot_get_device("m3_motor");
    WbDeviceTag front_left_motor = wb_robot_get_device("m4_motor");
    WbDeviceTag motors[4] = {front_right_motor, back_right_motor, back_left_motor, front_left_motor};

    const double time = wb_robot_get_time(); // in seconds

    wb_keyboard_enable(TIME_STEP);
    int key = wb_keyboard_get_key();
    while (key > 0){
        switch (key){
        case WB_KEYBOARD_UP:
            controller.pitch_disturbance = -2.0;
            break;
        case WB_KEYBOARD_DOWN:
            controller.pitch_disturbance = 2.0;
            break;
        case WB_KEYBOARD_RIGHT:
            controller.yaw_disturbance = -1.3;
            break;
        case WB_KEYBOARD_LEFT:
            controller.yaw_disturbance = 1.3;
            break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_RIGHT):
            controller.roll_disturbance = -1.0;
            break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_LEFT):
            controller.roll_disturbance = 1.0;
            break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP):
            pid.desired_altitude += 0.05;
            printf("desired altitude: %f [m]\n", pid.desired_altitude);
            break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
            pid.desired_altitude -= 0.05;
            printf("desired altitude: %f [m]\n", pid.desired_altitude);
            break;
        }
        key = wb_keyboard_get_key();
    }

    // calculate roll, pitch, yaw and altitude control input
    // control input of roll angle of quadcopter
    controller.control_roll = pid.kp_roll * CLAMP(ekf.estimated_state.roll, -1.0, 1.0) + ekf.state1.roll_angular_velocity + controller.roll_disturbance;
    archive.control_roll[i] = controller.control_roll;
    printf("control_roll: %lf\n", controller.control_roll);

    // control input of pitch angle of quadcopter
    controller.control_pitch = pid.kp_pitch * CLAMP(ekf.estimated_state.pitch, -1.0, 1.0) + ekf.state1.pitch_angular_velocity + controller.pitch_disturbance;
    archive.control_pitch[i] = controller.control_pitch;
    printf("control_pitch: %lf\n", controller.control_pitch);

    // control input of yaw angle of quadcopter
    controller.control_yaw = controller.yaw_disturbance;
    archive.control_yaw[i] = controller.control_yaw;
    printf("control_yaw: %lf\n", controller.control_yaw);

    // difference of altitude of drone from desired altitude
    controller.altitude_difference = CLAMP(pid.desired_altitude - ekf.estimated_state.position_z + controller.k_vertical_offset, -1.0, 1.0);
    archive.altitude_difference[i] = controller.altitude_difference;
    printf("altitude_difference: %lf\n", controller.altitude_difference);

    // control input of altitude of quadcopter
    controller.control_altitude = pid.kp_altitude * pow(controller.altitude_difference, 3.0);
    archive.control_altitude[i] = controller.control_altitude;
    printf("control_altitude: %lf\n", controller.control_altitude);

    // input value for front right motor of the quadcopter
    controller.front_right_motor_input = controller.k_vertical_thrust + controller.control_altitude + controller.control_roll + controller.control_pitch + controller.control_yaw;
    archive.front_right_motor_input[i] = controller.front_right_motor_input;
    printf("front_right_motor_input: %lf\n", controller.front_right_motor_input);

    // input value for back right motor of the quadcopter
    controller.back_right_motor_input = controller.k_vertical_thrust + controller.control_altitude + controller.control_roll - controller.control_pitch - controller.control_yaw;
    archive.back_right_motor_input[i] = controller.back_right_motor_input;
    printf("back_right_motor_input: %lf\n", controller.back_right_motor_input);

    // input value for back left motor of the quadcopter
    controller.back_left_motor_input = controller.k_vertical_thrust + controller.control_altitude - controller.control_roll - controller.control_pitch + controller.control_yaw;
    archive.back_left_motor_input[i] = controller.back_left_motor_input;
    printf("back_left_motor_input: %lf\n", controller.back_left_motor_input);

    // input value for front left motor of the quadcopter
    controller.front_left_motor_input = controller.k_vertical_thrust + controller.control_altitude - controller.control_roll + controller.control_pitch - controller.control_yaw;
    archive.front_left_motor_input[i] = controller.front_left_motor_input;
    printf("front_left_motor_input: %lf\n", controller.front_left_motor_input);

    wb_motor_set_velocity(front_right_motor, -controller.front_right_motor_input);
    wb_motor_set_velocity(back_right_motor, controller.back_right_motor_input);
    wb_motor_set_velocity(back_left_motor, -controller.back_left_motor_input);
    wb_motor_set_velocity(front_left_motor, controller.front_left_motor_input);

}
