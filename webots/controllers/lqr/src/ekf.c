#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include "src.h"
#include "savefile.h"
#include "ekf.h"

extern _archive archive;
_ekf ekf;

// state transition function
void state_transition_function(const state_variable *state, const double *control, const system_state *state1, state_variable *new_state){

    double f[n_state]; // linearized function
    // position state equation
    f[0] = state->velocity_i * cos(state->pitch) * cos(state->yaw) +
           state->velocity_j * (sin(state->roll) * sin(state->pitch) * cos(state->yaw) - cos(state->roll * sin(state->yaw))) +
           state->velocity_k * (cos(state->roll) * sin(state->pitch) * cos(state->yaw) + sin(state->roll * sin(state->yaw)));

    f[1] = state->velocity_i * cos(state->pitch) * sin(state->yaw) +
           state->velocity_j * (sin(state->roll) * sin(state->pitch) * sin(state->yaw) + cos(state->roll * cos(state->yaw))) +
           state->velocity_k * (cos(state->roll) * sin(state->pitch) * sin(state->yaw) - sin(state->roll * cos(state->yaw)));

    f[2] = state->velocity_i * (-sin(state->pitch)) + state->velocity_j * sin(state->roll) * cos(state->pitch) +
           state->velocity_k * cos(state->roll) * cos(state->pitch);

    // velocity represented in body frame state equation
    f[3] = - (state1->angular_velocity_z * state->velocity_j - state1->angular_velocity_y * state->velocity_k - g * sin(state->pitch));
    f[4] = - (state1->angular_velocity_x * state->velocity_k - state1->angular_velocity_z * state->velocity_i + g * cos(state->pitch) * sin(state->roll));
    f[5] = - (state1->angular_velocity_y * state->velocity_i - state1->angular_velocity_x * state->velocity_j + g * cos(state->pitch) * cos(state->roll) - (g + state1->acceleration_z));

    // euler angle state equation
    f[6] = state1->angular_velocity_x + state1->angular_velocity_y * sin(state->roll) * tan(state->pitch) + state1->angular_velocity_z * cos(state->roll) * tan(state->pitch);
    f[7] = state1->angular_velocity_y * cos(state->roll) - state1->angular_velocity_z * sin(state->roll);
    f[8] = state1->angular_velocity_y * sin(state->roll) / cos(state->pitch) + state1->angular_velocity_z * cos(state->roll) / cos(state->pitch);

    for (int j = 9; j < n_state; j++){
        f[j] = 0;
    }

    // printf("\nf:\n[");
    // printf("%.3lf", f[0]);
    // for (int j = 1; j < n_state; j++){
    //     printf(" %.3lf", f[j]);
    // }
    // printf("]\n");

    new_state->position_x = state->position_x + f[0] * Ts;               // x-axis position of drone
    new_state->position_y = state->position_y + f[1] * Ts;               // y-axis position of drone
    new_state->position_z = state->position_z + f[2] * Ts;               // z-axis position of drone
    new_state->velocity_i = state->velocity_i + f[3] * Ts;               // i-axis velocity of drone represented in body frame
    new_state->velocity_j = state->velocity_j + f[4] * Ts;               // j-axis velocity of drone represented in body frame
    new_state->velocity_k = state->velocity_k + f[5] * Ts;               // k-axis velocity of drone represented in body frame
    new_state->roll = state->roll + f[6] * Ts;                           // roll angle of drone
    new_state->pitch = state->pitch + f[7] * Ts;                         // pitch angle of drone
    new_state->yaw = state->yaw + f[8] * Ts;                             // yaw angle of drone
    new_state->bias_angular_velocity_x = state->bias_angular_velocity_x; // x-axis angular velocity bias of drone
    new_state->bias_angular_velocity_y = state->bias_angular_velocity_y; // y-axis angular velocity bias of drone
    new_state->bias_angular_velocity_z = state->bias_angular_velocity_z; // z-axis angular velocity bias of drone
    new_state->bias_acceleration_x = state->bias_acceleration_x;         // x-axis acceleration bias of drone
    new_state->bias_acceleration_y = state->bias_acceleration_y;         // y-axis acceleration bias of drone
    new_state->bias_acceleration_z = state->bias_acceleration_z;         // z-axis acceleration bias of drone
}

// observation function
void observation_function(const state_variable *state, const system_state *state1, measurement_variable *observation){
    observation->measurement_angular_velocity_x = state1->angular_velocity_x + state->bias_angular_velocity_x; // x-axis measurement angular velocity bias of drone
    observation->measurement_angular_velocity_y = state1->angular_velocity_y + state->bias_angular_velocity_y; // y-axis measurement angular velocity bias of drone
    observation->measurement_angular_velocity_z = state1->angular_velocity_z + state->bias_angular_velocity_z; // z-axis measurement angular velocity bias of drone
    observation->measurement_acceleration_x = state1->acceleration_x + state->bias_acceleration_x;             // x-axis measurement acceleration bias of drone
    observation->measurement_acceleration_y = state1->acceleration_y + state->bias_acceleration_y;             // y-axis measurement acceleration bias of drone
    observation->measurement_acceleration_z = state1->acceleration_z + state->bias_acceleration_z;             // z-axis measurement acceleration bias of drone
    observation->measurement_position_x = state->position_x;                                                   // x-axis measurement position of drone
    observation->measurement_position_y = state->position_y;                                                   // y-axis measurement position of drone
    observation->measurement_position_z = state->position_z;                                                   // z-axis measurement position of drone
    observation->measurement_yaw = state->yaw;                                                                 // measurement yaw angle of drone
}

// inertial frame converted to body frame
void InertialtoBody(const state_variable *state, const system_state *state1, state_variable *new_state, system_state *new_state1){

    // rotation matrix of body frame converted to inertial frame
    double R_BodytoInertial[3][3];
    R_BodytoInertial[0][0] = cos(state->pitch) * cos(state->yaw);
    R_BodytoInertial[0][1] = sin(state->roll) * sin(state->pitch) * cos(state->yaw) - cos(state->roll) * sin(state->yaw);
    R_BodytoInertial[0][2] = cos(state->roll) * sin(state->pitch) * cos(state->yaw) + sin(state->roll) * sin(state->yaw);

    R_BodytoInertial[1][0] = cos(state->pitch) * sin(state->yaw);
    R_BodytoInertial[1][1] = sin(state->roll) * sin(state->pitch) * sin(state->yaw) + cos(state->roll) * cos(state->yaw);
    R_BodytoInertial[1][2] = cos(state->roll) * sin(state->pitch) * sin(state->yaw) - sin(state->roll) * cos(state->yaw);

    R_BodytoInertial[2][0] = -sin(state->pitch);
    R_BodytoInertial[2][1] = sin(state->roll) * cos(state->pitch);
    R_BodytoInertial[2][2] = cos(state->roll) * cos(state->pitch);

    // printf("R_BodytoInertial matrix:\n");
    // for (int j = 0; j < 3; j++) {
    //     for (int k = 0; k < 3; k++) {
    //         printf("%f ", R_BodytoInertial[j][k]);
    //     }
    //     printf("\n");
    // }

    // rotation matrix of inertial frame converted to body frame
    double R_InertialtoBody[3][3];
    inv_matrix(3, R_BodytoInertial, R_InertialtoBody);

    new_state->velocity_i = R_InertialtoBody[0][0] * state1->velocity_x + R_InertialtoBody[0][1] * state1->velocity_y + R_InertialtoBody[0][2] * state1->velocity_z;
    new_state->velocity_j = R_InertialtoBody[1][0] * state1->velocity_x + R_InertialtoBody[1][1] * state1->velocity_y + R_InertialtoBody[1][2] * state1->velocity_z;
    new_state->velocity_k = R_InertialtoBody[2][0] * state1->velocity_x + R_InertialtoBody[2][1] * state1->velocity_y + R_InertialtoBody[2][2] * state1->velocity_z;

    // x-axis angular velocity of drone
    new_state1->angular_velocity_x = R_BodytoInertial[0][0] * state1->roll_angular_velocity + R_BodytoInertial[0][1] * state1->pitch_angular_velocity + R_BodytoInertial[0][2] * state1->yaw_angular_velocity;
    // y-axis angular velocity of drone
    new_state1->angular_velocity_y = R_BodytoInertial[1][0] * state1->roll_angular_velocity + R_BodytoInertial[1][1] * state1->pitch_angular_velocity + R_BodytoInertial[1][2] * state1->yaw_angular_velocity;
    // z-axis angular velocity of drone
    new_state1->angular_velocity_z = R_BodytoInertial[2][0] * state1->roll_angular_velocity + R_BodytoInertial[2][1] * state1->pitch_angular_velocity + R_BodytoInertial[2][2] * state1->yaw_angular_velocity;
}

void get_jacobian_F(const state_variable *state, const system_state *state1, double J[n_state][n_state]) {
    for (int j = 0; j < n_state; j++){
        for (int k = 0; k < n_state; k++){
            J[j][k] = 0.0;
        }
    }

    J[0][3] = cos(state->pitch) * cos(state->yaw);
    J[0][4] = cos(state->yaw) * sin(state->roll) * sin(state->pitch) - cos(state->roll) * sin(state->yaw);
    J[0][5] = sin(state->roll) * sin(state->yaw) + cos(state->roll) * cos(state->yaw) * sin(state->pitch);
    J[0][6] = state->velocity_j * (sin(state->roll) * sin(state->yaw) + cos(state->roll) * cos(state->yaw) * sin(state->pitch)) +
              state->velocity_k * (cos(state->roll) * sin(state->yaw) - cos(state->yaw) * sin(state->roll) * sin(state->pitch));
    J[0][7] = state->velocity_k * cos(state->roll) * cos(state->pitch) * cos(state->yaw) -
              state->velocity_i * cos(state->yaw) * sin(state->pitch) +
              state->velocity_j * cos(state->pitch) * cos(state->yaw) * sin(state->roll);
    J[0][8] = state->velocity_k * (cos(state->yaw) * sin(state->roll) - cos(state->roll) * sin(state->pitch) * sin(state->yaw)) -
              state->velocity_j * (cos(state->roll) * cos(state->yaw) + sin(state->roll) * sin(state->pitch) * sin(state->yaw)) -
              state->velocity_i * cos(state->pitch) * sin(state->yaw);

    J[1][3] = cos(state->pitch) * sin(state->yaw);
    J[1][4] = cos(state->roll) * cos(state->yaw) + sin(state->roll) * sin(state->pitch) * sin(state->yaw);
    J[1][5] = cos(state->roll) * sin(state->pitch) * sin(state->yaw) - cos(state->yaw) * sin(state->roll);
    J[1][6] = -state->velocity_j * (cos(state->yaw) * sin(state->roll) - cos(state->roll) * sin(state->pitch) * sin(state->yaw)) -
               state->velocity_k * (cos(state->roll) * cos(state->yaw) + sin(state->roll) * sin(state->pitch) * sin(state->yaw));
    J[1][7] = state->velocity_k * cos(state->roll) * cos(state->pitch) * sin(state->yaw) -
              state->velocity_i * sin(state->roll) * sin(state->yaw) +
              state->velocity_j * cos(state->pitch) * sin(state->yaw) * sin(state->roll);
    J[1][8] = state->velocity_k * (sin(state->roll) * sin(state->yaw) + cos(state->roll) * cos(state->yaw) * sin(state->pitch)) -
              state->velocity_j * (cos(state->roll) * sin(state->yaw) - cos(state->yaw) * sin(state->roll) * sin(state->pitch)) +
              state->velocity_i * cos(state->pitch) * cos(state->yaw);

    J[2][3] = -sin(state->pitch);
    J[2][4] = cos(state->pitch) * sin(state->roll);
    J[2][5] = cos(state->roll) * cos(state->pitch);
    J[2][6] = state->velocity_j * cos(state->roll) * cos(state->pitch) - state->velocity_k * cos(state->pitch) * sin(state->roll);
    J[2][7] = -state->velocity_i * cos(state->pitch) - state->velocity_k * cos(state->roll) * sin(state->pitch) - state->velocity_j * sin(state->roll) * sin(state->pitch);

    J[3][4] = state1->angular_velocity_z;
    J[3][5] = -state1->angular_velocity_y;
    J[3][8] = -g * cos(state->pitch);

    J[4][3] = -state1->angular_velocity_z;
    J[4][5] = state1->angular_velocity_x;
    J[4][6] = g * cos(state->roll) * cos(state->pitch);
    J[4][7] = -g * sin(state->roll) * sin(state->pitch);

    J[5][3] = state1->angular_velocity_y;
    J[5][4] = -state1->angular_velocity_x;
    J[5][6] = -g * cos(state->pitch) * sin(state->roll);
    J[5][7] = -g * cos(state->roll) * sin(state->pitch);

    J[6][6] = state1->angular_velocity_y * cos(state->pitch) * tan(state->roll) -
              state1->angular_velocity_z * sin(state->pitch) * tan(state->roll);
    J[6][7] = state1->angular_velocity_z * cos(state->pitch) * (pow(tan(state->roll), 2) + 1) +
              state1->angular_velocity_y * sin(state->pitch) * (pow(tan(state->roll), 2) + 1);

    J[7][6] = -state1->angular_velocity_z * cos(state->pitch) - state1->angular_velocity_y * sin(state->pitch);

    J[8][6] = (state1->angular_velocity_y * cos(state->pitch)) / cos(state->roll) -
              (state1->angular_velocity_z * sin(state->pitch)) / cos(state->roll);
    J[8][7] = (state1->angular_velocity_z * cos(state->pitch) * sin(state->roll)) / pow(cos(state->roll), 2) +
              (state1->angular_velocity_y * sin(state->pitch) * sin(state->roll)) / pow(cos(state->roll), 2);
}

void EKF_init(){
    // initialize state variable and covariance matrix
    // initialize true state variable
    for (int j = 0; j < n_state; j++){
        *((double *)&ekf.true_state + j) = 0;
    }

    // initialize estimated state variable
    for (int j = 0; j < n_state; j++){
        *((double *)&ekf.estimated_state + j) = 0;
    }

    // initialize covariance matrix P
    for (int j = 0; j < n_state; j++){
        for (int k = 0; k < n_state; k++){
            if (j == k){
                ekf.covariance[j][k] = 1;
            }
            else{
                ekf.covariance[j][k] = 0;
            }
        }
    }

    // process noise W covariance matrix Q
    for (int j = 0; j < n_state; j++){
        for (int k = 0; k < n_state; k++){
            if (j == k){
                ekf.Q[j][k] = 0.1;
            }
            else{
                ekf.Q[j][k] = 0;
            }
        }
    }

    // gaussian white noise V covariance matrix R
    for (int j = 0; j < n_measurement; j++){
        for (int k = 0; k < n_measurement; k++){
            if (j == k){
                ekf.R[j][k] = 1;
            }
            else{
                ekf.R[j][k] = 0;
            }
        }
    }

    ekf.last_position_x = 0.0;   // x-axis position of drone at last sampling moment
    ekf.last_position_y = 0.0;   // y-axis position of drone at last sampling moment
    ekf.last_position_z = 0.016; // z-axis position of drone at last sampling moment
    ekf.last_velocity_x = 0.0;   // x-axis velocity of drone at last sampling moment
    ekf.last_velocity_y = 0.0;   // y-axis velocity of drone at last sampling moment
    ekf.last_velocity_z = 0.0;   // z-axis velocity of drone at last sampling moment

    ekf.true_state.bias_angular_velocity_x = -0.01; // x-axis angular velocity bias of drone
    ekf.true_state.bias_angular_velocity_y = 0.01;  // y-axis angular velocity bias of drone
    ekf.true_state.bias_angular_velocity_z = -0.01; // z-axis angular velocity bias of drone
    ekf.true_state.bias_acceleration_x = 0.01;      // x-axis acceleration bias of drone
    ekf.true_state.bias_acceleration_y = -0.01;     // y-axis acceleration bias of drone
    ekf.true_state.bias_acceleration_z = -0.01;     // z-axis acceleration bias of drone
}

// extended kalman filter
void EKF_realize(int i){

    double control = 0.0;

    /*********** update true state variable ***********/
    // state_transition_function(&ekf.true_state, &control, &ekf.true_state);
    // obtain altitude of UAV using GPS sensor
    WbDeviceTag gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);
    const double *coordinate = wb_gps_get_values(gps);

    ekf.true_state.position_x = coordinate[0]; // x-axis position of drone
    if (i == 0){
        ekf.true_state.position_x = 0.0;
    }
    archive.position_x[i] = ekf.true_state.position_x;
    printf("x position: %lf\n", ekf.true_state.position_x);

    ekf.true_state.position_y = coordinate[1]; // y-axis position of drone
    if (i == 0){
        ekf.true_state.position_y = 0.0;
    }
    archive.position_y[i] = ekf.true_state.position_y;
    printf("y position: %lf\n", ekf.true_state.position_y);

    ekf.true_state.position_z = coordinate[2]; // z-axis position of drone
    if (i == 0){
        ekf.true_state.position_z = 0.065;
    }
    archive.position_z[i] = ekf.true_state.position_z;
    printf("z position: %lf\n", ekf.true_state.position_z);

    ekf.state1.velocity_x = (ekf.true_state.position_x - ekf.last_position_x) / Ts; // x-axis velocity of drone
    archive.velocity_x[i] = ekf.state1.velocity_x;
    printf("x velocity: %lf\n", ekf.state1.velocity_x);

    ekf.state1.velocity_y = (ekf.true_state.position_y - ekf.last_position_y) / Ts; // y-axis velocity of drone
    archive.velocity_y[i] = ekf.state1.velocity_y;
    printf("y velocity: %lf\n", ekf.state1.velocity_y);

    ekf.state1.velocity_z = (ekf.true_state.position_z - ekf.last_position_z) / Ts; // z-axis velocity of drone
    archive.velocity_z[i] = ekf.state1.velocity_z;
    printf("z velocity: %lf\n", ekf.state1.velocity_z);

    ekf.state1.acceleration_x = (ekf.state1.velocity_x - ekf.last_velocity_x) / Ts; // x-axis acceleration of drone
    archive.acceleration_x[i] = ekf.state1.acceleration_x;
    printf("x acceleration: %lf\n", ekf.state1.acceleration_x);

    ekf.state1.acceleration_y = (ekf.state1.velocity_y - ekf.last_velocity_y) / Ts; // y-axis acceleration of drone
    archive.acceleration_y[i] = ekf.state1.acceleration_y;
    printf("y acceleration: %lf\n", ekf.state1.acceleration_y);

    ekf.state1.acceleration_z = (ekf.state1.velocity_z - ekf.last_velocity_z) / Ts; // z-axis acceleration of drone
    archive.acceleration_z[i] = ekf.state1.acceleration_z;
    printf("z acceleration: %lf\n", ekf.state1.acceleration_z);

    // get three Euler angles of UAV sing inertial measurement unit sensor
    WbDeviceTag imu = wb_robot_get_device("inertial_unit");
    wb_inertial_unit_enable(imu, TIME_STEP);
    const double *roll_pitch_yaw = wb_inertial_unit_get_roll_pitch_yaw(imu);

    ekf.true_state.roll = roll_pitch_yaw[0]; // roll angle of drone
    archive.roll[i] = ekf.true_state.roll;
    printf("roll angle: %lf\n", ekf.true_state.roll);

    ekf.true_state.pitch = roll_pitch_yaw[1]; // pitch angle of drone
    archive.pitch[i] = ekf.true_state.pitch;
    printf("pitch angle: %lf\n", ekf.true_state.pitch);

    ekf.true_state.yaw = roll_pitch_yaw[2]; // yaw angle of drone
    archive.yaw[i] = ekf.true_state.yaw;
    printf("yaw angle: %lf\n", ekf.true_state.yaw);

    // inertial frame converted to body frame
    InertialtoBody(&ekf.true_state, &ekf.state1, &ekf.true_state, &ekf.state1);

    // i-axis velocity of drone represented in body frame
    archive.velocity_i[i] = ekf.true_state.velocity_i;
    printf("velocity_i: %lf\n", ekf.true_state.velocity_i);

    // j-axis velocity of drone represented in body frame
    archive.velocity_j[i] = ekf.true_state.velocity_j;
    printf("velocity_j: %lf\n", ekf.true_state.velocity_j);

    // k-axis velocity of drone represented in body frame
    archive.velocity_k[i] = ekf.true_state.velocity_k;
    printf("velocity_k: %lf\n", ekf.true_state.velocity_k);

    //  x-axis angular velocity of drone
    archive.angular_velocity_x[i] = ekf.state1.angular_velocity_x;
    printf("x angular velocity: %lf\n", ekf.state1.angular_velocity_x);

    //  y-axis angular velocity of drone
    archive.angular_velocity_y[i] = ekf.state1.angular_velocity_y;
    printf("y angular velocity: %lf\n", ekf.state1.angular_velocity_y);

    //  z-axis angular velocity of drone
    archive.angular_velocity_z[i] = ekf.state1.angular_velocity_z;
    printf("z angular velocity: %lf\n", ekf.state1.angular_velocity_z);

    /*********** prediction step ***********/
    if (i == 0){
        ekf.estimated_state = ekf.true_state;
    }

    // obtain three euler angular velocity of UAV using gyroscopic sensor
    WbDeviceTag gyro = wb_robot_get_device("gyro");
    wb_gyro_enable(gyro, TIME_STEP);
    const double *gyro_value = wb_gyro_get_values(gyro);

    ekf.state1.roll_angular_velocity = gyro_value[0]; // roll angular velocity of the drone
    if (i == 0){
        ekf.state1.roll_angular_velocity = 0.0;
    }
    archive.roll_angular_velocity[i] = ekf.state1.roll_angular_velocity;
    printf("roll angular velocity: %lf\n", ekf.state1.roll_angular_velocity);

    ekf.state1.pitch_angular_velocity = gyro_value[1]; // pitch angular velocity of the drone
    if (i == 0){
        ekf.state1.pitch_angular_velocity = 0.0;
    }
    archive.pitch_angular_velocity[i] = ekf.state1.pitch_angular_velocity;
    printf("pitch angular velocity: %lf\n", ekf.state1.pitch_angular_velocity);

    ekf.state1.yaw_angular_velocity = gyro_value[2]; // yaw angular velocity of the drone
    if (i == 0){
        ekf.state1.yaw_angular_velocity = 0.0;
    }
    archive.yaw_angular_velocity[i] = ekf.state1.yaw_angular_velocity;
    printf("yaw angular velocity: %lf\n", ekf.state1.yaw_angular_velocity);

    printf("\nstate1:\n[");
    printf("%.3lf", ekf.state1.angular_velocity_x);
    for (int j = 1; j < n_state; j++){
        printf(" %.3lf", *((double *)&ekf.state1 + j));
    }
    printf("]\n");

    if (ekf.true_state.position_z > 0.016){
        // predicted state variabl is equal to state transition matrix multiplied by estimated state variable
        state_transition_function(&ekf.estimated_state, &control, &ekf.state1, &ekf.predicted_state);
    } if (ekf.true_state.position_z <= 0.016){
        ekf.estimated_state = ekf.true_state;
        ekf.predicted_state = ekf.estimated_state;
    }

    // jacobian matrice of nonlinear function f on state variable x, Phi
    get_jacobian_F(&ekf.estimated_state, &ekf.state1, ekf.jacobian_F);
    // printf("jacobian_F:\n");
    // for (int j = 0; j < n_state; j++) {
    //     for (int k = 0; k < n_state; k++) {
    //         printf("%.3f ", ekf.jacobian_F[j][k]);
    //     }
    //     printf("\n");
    // }

    // transpose matrix of state transfer matrix, Phi'
    double jacobian_F_transpose[n_state][n_state];
    matrix_transpose(n_state, n_state, ekf.jacobian_F, jacobian_F_transpose);

    // compute jacobian matrix respect to f multiplied by covariance matrix P
    double jacobian_F_covariance[n_state][n_state];
    matrix_multi((double *)jacobian_F_covariance, (double *)ekf.jacobian_F, (double *)ekf.covariance, n_state, n_state, n_state);
    // printf("jacobian_F_covariance:\n");
    // for (int j = 0; j < n_state; j++) {
    //     for (int k = 0; k < n_state; k++) {
    //         printf("%.3f ", jacobian_F_covariance[j][k]);
    //     }
    //     printf("\n");
    // }

    // then multiply by transpose of jacobian matrix respect to f
    double temp[n_state][n_state];
    matrix_multi((double *)temp, (double *)jacobian_F_covariance, (double *)jacobian_F_transpose, n_state, n_state, n_state);

    /* predicted covariance matrix is equal to jacobian matrix about f multiplied by covariance matrix P multiplied
    by transpose of jacobian matrix about f plus noise variance Q*/
    for (int j = 0; j < n_state; j++){
        for (int k = 0; k < n_state; k++){
            ekf.predicted_covariance[j][k] = temp[j][k] + ekf.Q[j][k];
        }
    }
    // printf("predicted covariance:\n");
    // for (int j = 0; j < n_state; j++) {
    //     for (int k = 0; k < n_state; k++) {
    //         printf("%.3f ", ekf.predicted_covariance[j][k]);
    //     }
    //     printf("\n");
    // }

    /*********** update step ***********/
    observation_function(&ekf.true_state, &ekf.state1, &ekf.measurement); // observation value Z of state variable, considering sensor error
    ekf.measurement.measurement_angular_velocity_x += 0.01 * (rand() / (double)RAND_MAX);
    ekf.measurement.measurement_angular_velocity_y += 0.01 * (rand() / (double)RAND_MAX);
    ekf.measurement.measurement_angular_velocity_z += 0.01 * (rand() / (double)RAND_MAX);
    ekf.measurement.measurement_acceleration_x += 0.01 * (rand() / (double)RAND_MAX);
    ekf.measurement.measurement_acceleration_y += 0.01 * (rand() / (double)RAND_MAX);
    ekf.measurement.measurement_acceleration_z += 0.01 * (rand() / (double)RAND_MAX);
    ekf.measurement.measurement_position_x += 0.01 * (rand() / (double)RAND_MAX);
    ekf.measurement.measurement_position_y += 0.01 * (rand() / (double)RAND_MAX);
    ekf.measurement.measurement_position_z += 0.01 * (rand() / (double)RAND_MAX);
    ekf.measurement.measurement_yaw += 0.01 * (rand() / (double)RAND_MAX);

    // observation matrix H, i.e., jacobian matrix of observation function h for state variable x
    ekf.jacobian_H[0][9] = 1;
    ekf.jacobian_H[1][10] = 1;
    ekf.jacobian_H[2][11] = 1;
    ekf.jacobian_H[3][12] = 1;
    ekf.jacobian_H[4][13] = 1;
    ekf.jacobian_H[5][14] = 1;
    ekf.jacobian_H[6][0] = 1;
    ekf.jacobian_H[7][1] = 1;
    ekf.jacobian_H[8][2] = 1;
    ekf.jacobian_H[9][8] = 1;

    // printf("jacobian_H:\n");
    // for (int j = 0; j < n_measurement; j++) {
    //     for (int k = 0; k < n_state; k++) {
    //         printf("%.3f ", ekf.jacobian_H[j][k]);
    //     }
    //     printf("\n");
    // }

    // transpose matrixof jacobian matrix with respect to observed function h, H'
    double jacobian_H_transpose[n_state][n_measurement];
    matrix_transpose(n_measurement, n_state, ekf.jacobian_H, jacobian_H_transpose);

    // compute jacobian matrix with respect to observed function h multiplied by predicted covariance matrix
    double jacobian_H_pre_covariance[n_measurement][n_state];
    matrix_multi((double *)jacobian_H_pre_covariance, (double *)ekf.jacobian_H, (double *)ekf.predicted_covariance, n_measurement, n_state, n_state);

    // then multiply by transpose of jacobian matrix respect to h
    double temp1[n_measurement][n_measurement];
    matrix_multi((double *)temp1, (double *)jacobian_H_pre_covariance, (double *)jacobian_H_transpose, n_measurement, n_state, n_measurement);

    /* temporary matrix S is equal to jacobian matrix about h multiplied by predicted covariance matrix multiplied
    by transpose of jacobian matrix about h plus noise variance R */
    double S[n_measurement][n_measurement];
    for (int j = 0; j < n_measurement; j++){
        for (int k = 0; k < n_measurement; k++){
            S[j][k] = temp1[j][k] + ekf.R[j][k];
        }
    }

    double inv_S[n_measurement][n_measurement]; // compute inverse matrix of temporary matrix S
    inv_matrix(n_measurement, S, inv_S);

    // compute predicted covariance matrix multiplied by transpose matrix of jacobian matrix about h
    double pre_covariance_jacobian_Ht[n_state][n_measurement];
    matrix_multi((double *)pre_covariance_jacobian_Ht, (double *)ekf.predicted_covariance, (double *)jacobian_H_transpose, n_state, n_state, n_measurement);

    /* kalman gain K is equal to predicted covariance matrix multiplied by transpose matrix of jacobian matrix about h multiplied
    by transpose of temporary matrix */
    matrix_multi((double *)ekf.K, (double *)pre_covariance_jacobian_Ht, (double *)inv_S, n_state, n_measurement, n_measurement);
    // printf("kalman gain K:\n");
    // for (int j = 0; j < n_state; j++) {
    //     for (int k = 0; k < n_measurement; k++) {
    //         printf("%.3f ", ekf.K[j][k]);
    //     }
    //     printf("\n");
    // }

    // observation minus observation of estimated predicted state
    double innovation[n_measurement];
    observation_function(&ekf.predicted_state, &ekf.state1, &ekf.measurement_predicted);
    innovation[0] = ekf.measurement.measurement_angular_velocity_x - ekf.measurement_predicted.measurement_angular_velocity_x;
    innovation[1] = ekf.measurement.measurement_angular_velocity_y - ekf.measurement_predicted.measurement_angular_velocity_y;
    innovation[2] = ekf.measurement.measurement_angular_velocity_z - ekf.measurement_predicted.measurement_angular_velocity_z;
    innovation[3] = ekf.measurement.measurement_acceleration_x - ekf.measurement_predicted.measurement_acceleration_x;
    innovation[4] = ekf.measurement.measurement_acceleration_y - ekf.measurement_predicted.measurement_acceleration_y;
    innovation[5] = ekf.measurement.measurement_acceleration_z - ekf.measurement_predicted.measurement_acceleration_z;
    innovation[6] = ekf.measurement.measurement_position_x - ekf.measurement_predicted.measurement_position_x;
    innovation[7] = ekf.measurement.measurement_position_y - ekf.measurement_predicted.measurement_position_y;
    innovation[8] = ekf.measurement.measurement_position_z - ekf.measurement_predicted.measurement_position_z;
    innovation[9] = ekf.measurement.measurement_yaw - ekf.measurement_predicted.measurement_yaw;

    // printf("innovation:\n");
    // for (int j = 0; j < n_measurement; j++) {
    //     printf("%.4f ", innovation[j]);
    // }
    // printf("\n");

    // compute kalman gain multiplied by observation minus observation of estimated predicted state
    double K_innovation[n_state];
    matrix_multi((double *)K_innovation, (double *)ekf.K, (double *)innovation, n_state, n_measurement, 1);

    // updated state variable is equal to predicted state variable plus kalman gain multiplied by observation minus observation of estimated predicted state
    ekf.updated_state.position_x = ekf.predicted_state.position_x + K_innovation[0];
    ekf.updated_state.position_y = ekf.predicted_state.position_y + K_innovation[1];
    ekf.updated_state.position_z = ekf.predicted_state.position_z + K_innovation[2];
    ekf.updated_state.velocity_i = ekf.predicted_state.velocity_i + K_innovation[3];
    ekf.updated_state.velocity_j = ekf.predicted_state.velocity_j + K_innovation[4];
    ekf.updated_state.velocity_k = ekf.predicted_state.velocity_k + K_innovation[5];
    ekf.updated_state.roll = ekf.predicted_state.roll + K_innovation[6];
    ekf.updated_state.pitch = ekf.predicted_state.pitch + K_innovation[7];
    ekf.updated_state.yaw = ekf.predicted_state.yaw + K_innovation[8];
    ekf.updated_state.bias_angular_velocity_x = ekf.predicted_state.bias_angular_velocity_x + K_innovation[9];
    ekf.updated_state.bias_angular_velocity_y = ekf.predicted_state.bias_angular_velocity_y + K_innovation[10];
    ekf.updated_state.bias_angular_velocity_z = ekf.predicted_state.bias_angular_velocity_z + K_innovation[11];
    ekf.updated_state.bias_acceleration_x = ekf.predicted_state.bias_acceleration_x + K_innovation[12];
    ekf.updated_state.bias_acceleration_y = ekf.predicted_state.bias_acceleration_y + K_innovation[13];
    ekf.updated_state.bias_acceleration_z = ekf.predicted_state.bias_acceleration_z + K_innovation[14];

    // compute kalman gain multiplied by jacobian matrix with respect to observed function h
    double K_jacobian_H[n_state][n_state];
    matrix_multi((double *)K_jacobian_H, (double *)ekf.K, (double *)ekf.jacobian_H, n_state, n_measurement, n_state);

    double Identity[n_state][n_state]; // identity matrix I

    for (int j = 0; j < n_state; j++) {
        for (int k = 0; k < n_state; k++) {
            if (j == k) {
                Identity[j][k] = 1;
            } else {
                Identity[j][k] = 0;
            }
        }
    }

    /* identity matrix I minus kalman gain multiplied by jacobian matrix with respect to observed function h*/
    double I_K_jacobian_H[n_state][n_state];
    for (int j = 0; j < n_state; j++){
        for (int k = 0; k < n_state; k++){
            I_K_jacobian_H[j][k] = Identity[j][k] - K_jacobian_H[j][k];
        }
    }

    /* updated covariance matrix is equal to identity matrix I minus kalman gain K multiplied by jacobian matrixwith respect to
    observed function h, multiplied by predicted covariance matrix*/
    matrix_multi((double *)ekf.updated_covariance, (double *)I_K_jacobian_H, (double *)ekf.predicted_covariance, n_state, n_state, n_state);

    // update filter state variable and covariance matrix
    if (ekf.true_state.position_z > 0.016){
        ekf.estimated_state = ekf.updated_state;
    } else if (ekf.true_state.position_z <= 0.016){
        ekf.estimated_state = ekf.estimated_state;
    }

    for (int j = 0; j < n_state; j++){
        for (int k = 0; k < n_state; k++){
            ekf.covariance[j][k] = ekf.updated_covariance[j][k];
        }
    }

    printf("covariance matrix:\n");
    for (int j = 0; j < n_state; j++) {
        for (int k = 0; k < n_state; k++) {
            printf("%-5.2f ", ekf.covariance[j][k]);
        }
        printf("\n");
    }

    printf("\ntrue state:\n[");
    printf("%.3lf", ekf.true_state.position_x);
    for (int j = 1; j < n_state; j++){
        printf(" %.3lf", *((double *)&ekf.true_state + j));
    }
    printf("]\n");

    printf("\nestimated state:\n[");
    printf("%.3lf", ekf.estimated_state.position_x);
    for (int j = 1; j < n_state; j++){
        printf(" %.3lf", *((double *)&ekf.estimated_state + j));
    }
    printf("]\n");

    printf("\npredicted state:\n[");
    printf("%.3lf", ekf.predicted_state.position_x);
    for (int j = 1; j < n_state; j++){
        printf(" %.3lf", *((double *)&ekf.predicted_state + j));
    }
    printf("]\n");

    ekf.estimated_error.position_x = ekf.true_state.position_x - ekf.estimated_state.position_x;
    ekf.estimated_error.position_y = ekf.true_state.position_y - ekf.estimated_state.position_y;
    ekf.estimated_error.position_z = ekf.true_state.position_z - ekf.estimated_state.position_z;
    ekf.estimated_error.velocity_i = ekf.true_state.velocity_i - ekf.estimated_state.velocity_i;
    ekf.estimated_error.velocity_j = ekf.true_state.velocity_j - ekf.estimated_state.velocity_j;
    ekf.estimated_error.velocity_k = ekf.true_state.velocity_k - ekf.estimated_state.velocity_k;
    ekf.estimated_error.roll = ekf.true_state.roll - ekf.estimated_state.roll;
    ekf.estimated_error.pitch = ekf.true_state.pitch - ekf.estimated_state.pitch;
    ekf.estimated_error.yaw = ekf.true_state.yaw - ekf.estimated_state.yaw;
    ekf.estimated_error.bias_angular_velocity_x = ekf.true_state.bias_angular_velocity_x - ekf.estimated_state.bias_angular_velocity_x;
    ekf.estimated_error.bias_angular_velocity_y = ekf.true_state.bias_angular_velocity_y - ekf.estimated_state.bias_angular_velocity_y;
    ekf.estimated_error.bias_angular_velocity_z = ekf.true_state.bias_angular_velocity_z - ekf.estimated_state.bias_angular_velocity_z;
    ekf.estimated_error.bias_acceleration_x = ekf.true_state.bias_acceleration_x - ekf.estimated_state.bias_acceleration_x;
    ekf.estimated_error.bias_acceleration_y = ekf.true_state.bias_acceleration_y - ekf.estimated_state.bias_acceleration_y;
    ekf.estimated_error.bias_acceleration_z = ekf.true_state.bias_acceleration_z - ekf.estimated_state.bias_acceleration_z;

    printf("\nestimated error state:\n[");
    printf("%.3lf", ekf.estimated_error.position_x);
    for (int j = 1; j < n_state; j++){
        printf(" %.3lf", *((double *)&ekf.estimated_error + j));
    }
    printf("]\n");

    ekf.last_position_x = ekf.true_state.position_x;
    ekf.last_position_y = ekf.true_state.position_y;
    ekf.last_position_z = ekf.true_state.position_z;
    ekf.last_velocity_x = ekf.state1.velocity_x;
    ekf.last_velocity_y = ekf.state1.velocity_y;
    ekf.last_velocity_z = ekf.state1.velocity_z;
}
