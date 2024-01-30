#include <stdio.h>

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
#include "lqr.h"

extern _archive archive;
extern _ekf ekf;
extern _controller controller;
extern _pid pid;
_lqr lqr;
_sda sda;

void get_A(const state_variable *state, const system_state *state1, double A[n_lqr_state][n_lqr_state]) {
    for (int j = 0; j < n_lqr_state; j++){
        for (int k = 0; k < n_lqr_state; k++){
            A[j][k] = 0.0;
        }
    }

    A[0][0] = state1->angular_velocity_y * cos(state->roll) * tan(state->pitch) - state1->angular_velocity_z * tan(state->pitch) * sin(state->roll);
    A[0][1] = state1->angular_velocity_z * cos(state->roll) * (pow(tan(state->pitch), 2) + 1) +
              state1->angular_velocity_y * sin(state->roll) * (tan(state->pitch) * tan(state->pitch) + 1);

    A[0][3] = 1;
    A[0][4] = tan(state->pitch) * sin(state->roll);
    A[0][5] = cos(state->roll) * tan(state->pitch);

    A[1][0] = -state1->angular_velocity_z * cos(state->roll) - state1->angular_velocity_y * sin(state->roll);
    A[1][4] = cos(state->roll);
    A[1][5] = -sin(state->roll);

    A[2][0] = (state1->angular_velocity_y * cos(state->roll)) / cos(state->pitch) -
              (state1->angular_velocity_z * sin(state->roll)) / cos(state->pitch);
    A[2][1] = (state1->angular_velocity_z * cos(state->roll) * sin(state->pitch)) / pow(cos(state->pitch), 2) +
              (state1->angular_velocity_y * sin(state->pitch) * sin(state->roll)) / pow(cos(state->pitch), 2);
    A[2][4] = sin(state->roll) / cos(state->pitch);
    A[2][5] = cos(state->roll) / cos(state->pitch);

    A[3][4] = (state1->angular_velocity_z * (I_y - I_z)) / I_x;
    A[3][5] = (state1->angular_velocity_y * (I_y - I_z)) / I_x;

    A[4][3] = -(state1->angular_velocity_z * (I_x - I_z)) / I_y;
    A[4][5] = -(state1->angular_velocity_x * (I_x - I_z)) / I_y;

    A[5][3] = (state1->angular_velocity_y * (I_x - I_y)) / I_z;
    A[5][4] = (state1->angular_velocity_x * (I_x - I_y)) / I_z;

    A[6][1] = -g * cos(state->pitch);
    A[6][4] = -state->velocity_k;
    A[6][5] = state->velocity_j;
    A[6][7] = state1->angular_velocity_z;
    A[6][8] = -state1->angular_velocity_y;

    A[7][0] = g * cos(state->pitch) * cos(state->roll);
    A[7][1] = -g * sin(state->pitch) * sin(state->roll);
    A[7][3] = state->velocity_k;
    A[7][5] = -state->velocity_i;
    A[7][6] = -state1->angular_velocity_z;
    A[7][8] = state1->angular_velocity_x;

    A[8][0] = -g * cos(state->pitch) * sin(state->roll);
    A[8][1] = -g * cos(state->roll) * sin(state->pitch);
    A[8][3] = -state->velocity_j;
    A[8][4] = state->velocity_i;
    A[8][6] = state1->angular_velocity_y;
    A[8][7] = -state1->angular_velocity_x;

    A[9][0] = state->velocity_j * (sin(state->roll) * sin(state->yaw) + cos(state->roll) * cos(state->yaw) * sin(state->pitch)) +
              state->velocity_k * (cos(state->roll) * sin(state->yaw) - cos(state->yaw) * sin(state->pitch) * sin(state->roll));
    A[9][1] = state->velocity_k * cos(state->pitch) * cos(state->roll) * cos(state->yaw) -
              state->velocity_i * cos(state->yaw) * sin(state->pitch) +
              state->velocity_j * cos(state->pitch) * cos(state->yaw) * sin(state->roll);
    A[9][2] = state->velocity_k * (cos(state->yaw) * sin(state->roll) - cos(state->roll) * sin(state->pitch) * sin(state->yaw)) -
              state->velocity_j * (cos(state->roll) * cos(state->yaw) + sin(state->pitch) * sin(state->roll) * sin(state->yaw)) -
              state->velocity_i * cos(state->pitch) * sin(state->yaw);
    A[9][6] = cos(state->pitch) * cos(state->yaw);
    A[9][7] = cos(state->yaw) * sin(state->pitch) * sin(state->roll) - cos(state->roll) * sin(state->yaw);
    A[9][8] = sin(state->roll) * sin(state->yaw) + cos(state->roll) * cos(state->yaw) * sin(state->pitch);

    A[10][0] = state->velocity_k * (cos(state->roll) * cos(state->yaw) - sin(state->pitch) * sin(state->roll) * sin(state->yaw)) -
               state->velocity_j * (cos(state->yaw) * sin(state->roll) - cos(state->roll) * sin(state->pitch) * sin(state->yaw));
    A[10][1] = state->velocity_k * cos(state->pitch) * cos(state->roll) * sin(state->yaw) -
               state->velocity_i * sin(state->pitch) * sin(state->yaw) +
               state->velocity_j * cos(state->pitch) * sin(state->roll) * sin(state->yaw);
    A[10][2] = state->velocity_i * cos(state->pitch) * cos(state->yaw) -
               state->velocity_k * (sin(state->roll) * sin(state->yaw) - cos(state->roll) * cos(state->yaw) * sin(state->pitch)) -
               state->velocity_j * (cos(state->roll) * sin(state->yaw) - cos(state->yaw) * sin(state->pitch) * sin(state->roll));
    A[10][6] = cos(state->pitch) * sin(state->yaw);
    A[10][7] = cos(state->roll) * cos(state->yaw) + sin(state->pitch) * sin(state->roll) * sin(state->yaw);
    A[10][8] = cos(state->yaw) * sin(state->roll) + cos(state->roll) * sin(state->pitch) * sin(state->yaw);

    A[11][0] = state->velocity_j * cos(state->pitch) * cos(state->roll) - state->velocity_k * cos(state->pitch) * sin(state->roll);
    A[11][1] = -state->velocity_i * cos(state->pitch) - state->velocity_k * cos(state->roll) * sin(state->pitch) - state->velocity_j * sin(state->pitch) * sin(state->roll);
    A[11][6] = -sin(state->pitch);
    A[11][7] = cos(state->pitch) * sin(state->roll);
    A[11][8] = cos(state->pitch) * cos(state->roll);

}

void get_lqr_state(const state_variable *state, const system_state *state1, double state_lqr[n_lqr_state]){
    state_lqr[0] = state->roll;                // roll angle of drone
    state_lqr[1] = state->pitch;               // pitch angle of drone
    state_lqr[2] = state->yaw;                 // yaw angle of drone
    state_lqr[3] = state1->angular_velocity_x; // x-axis angular velocity of drone
    state_lqr[4] = state1->angular_velocity_y; // y-axis angular velocity of drone
    state_lqr[5] = state1->angular_velocity_z; // z-axis angular velocity of drone
    state_lqr[6] = state->velocity_i;          // i-axis velocity of drone represented in body frame
    state_lqr[7] = state->velocity_j;          // j-axis velocity of drone represented in body frame
    state_lqr[8] = state->velocity_k;          // k-axis velocity of drone represented in body frame
    state_lqr[9] = state->position_x;          // x-axis position of drone
    state_lqr[10] = state->position_y;         // y-axis position of drone
    state_lqr[11] = state->position_z;         // z-axis position of drone
}

// Structure-preserving Doubling Algorithm
void care_sda(const _lqr *lqr, _sda *sda){

    double I[n_lqr_state][n_lqr_state]; // identity matrix I

    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            if (j == k) {
                I[j][k] = 1.0;
            } else {
                I[j][k] = 0.0;
            }
        }
    }

    /* A_gamma = A - (gamma*I) */
    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            sda->A_gamma[j][k] = lqr->A[j][k] - sda->gamma * I[j][k];
        }
    }
    printf("A_gamma:\n");
    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            printf("%-5.2f ", sda->A_gamma[j][k]);
        }
        printf("\n");
    }

    double A_gamma_T[n_lqr_state][n_lqr_state]; // transpose matrix of A_gamma
    matrix_transpose(n_lqr_state, n_lqr_state, sda->A_gamma, A_gamma_T);

    double inv_A_gamma[n_lqr_state][n_lqr_state]; // inverse matrix of A_gamma
    inv_matrix(n_lqr_state, sda->A_gamma, inv_A_gamma);

    /******** calculate A_hat_last *******/
    // compute inverse matrix of transpose matrix of A_gamma
    double inv_A_gamma_T[n_lqr_state][n_lqr_state];
    inv_matrix(n_lqr_state, A_gamma_T, inv_A_gamma_T);

    // G multiplied by inverse matrix of transpose matrix of A_gamma
    double temp2[n_lqr_state][n_lqr_state];
    matrix_multi((double *)temp2, (double *)lqr->G, (double *)inv_A_gamma_T, n_lqr_state, n_lqr_state, n_lqr_state);

    /* G*inv(A_gamma_T)*H */
    double temp3[n_lqr_state][n_lqr_state];
    matrix_multi((double *)temp3, (double *)temp2, (double *)lqr->H, n_lqr_state, n_lqr_state, n_lqr_state);

    /* A_gamma + G*inv(A_gamma_T)*H */
    double temp4[n_lqr_state][n_lqr_state];
    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            temp4[j][k] = sda->A_gamma[j][k] + temp3[j][k];
        }
    }

    /* inv(A_gamma + G*inv(A_gamma_T)*H) */
    double temp5[n_lqr_state][n_lqr_state];
    inv_matrix(n_lqr_state, temp4, temp5);

    /* A_hat_last = I + 2*gamma*inv(A_gamma + G*inv(A_gamma_T)*H); */
    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            sda->A_hat_last[j][k] = I[j][k] + 2.0 * sda->gamma * temp5[j][k];
        }
    }
    printf("A_hat_last:\n");
    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            printf("%-6.2f ", sda->A_hat_last[j][k]);
        }
        printf("\n");
    }

    /******** calculate G_hat_last *******/
    // compute H multiplied by inverse matrix of A_gamma
    double temp6[n_lqr_state][n_lqr_state];
    matrix_multi((double *)temp6, (double *)lqr->H, (double *)inv_A_gamma, n_lqr_state, n_lqr_state, n_lqr_state);

    /* H*inv_A_gamma*G */
    double temp7[n_lqr_state][n_lqr_state];
    matrix_multi((double *)temp7, (double *)temp6, (double *)lqr->G, n_lqr_state, n_lqr_state, n_lqr_state);

    /* A_gamma_T + H*inv_A_gamma*G */
    double temp8[n_lqr_state][n_lqr_state];
    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            temp8[j][k] = A_gamma_T[j][k] + temp7[j][k];
        }
    }

    /* inv(A_gamma_T + H*inv_A_gamma*G) */
    double temp9[n_lqr_state][n_lqr_state];
    inv_matrix(n_lqr_state, temp8, temp9);

    /* inv_A_gamma*G */
    double temp10[n_lqr_state][n_lqr_state];
    matrix_multi((double *)temp10, (double *)inv_A_gamma, (double *)lqr->G, n_lqr_state, n_lqr_state, n_lqr_state);

    /* inv_A_gamma*G * inv(A_gamma_T + H*inv_A_gamma*G) */
    double temp11[n_lqr_state][n_lqr_state];
    matrix_multi((double *)temp11, (double *)temp10, (double *)temp9, n_lqr_state, n_lqr_state, n_lqr_state);

    /* G_hat_last = 2*gamma*inv_A_gamma*G * inv(A_gamma_T + H*inv_A_gamma*G) */
    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            sda->G_hat_last[j][k] = 2.0 * sda->gamma * temp11[j][k];
        }
    }
    printf("G_hat_last:\n");
    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            printf("%-6.2f ", sda->G_hat_last[j][k]);
        }
        printf("\n");
    }

    /******** calculate H_hat_last *******/
    /* inv(A_gamma_T + H*inv_A_gamma*G)*H */
    double temp12[n_lqr_state][n_lqr_state];
    matrix_multi((double *)temp12, (double *)temp9, (double *)lqr->H, n_lqr_state, n_lqr_state, n_lqr_state);

    /* inv(A_gamma_T + H*inv_A_gamma*G)*H*inv_A_gamma */
    double temp13[n_lqr_state][n_lqr_state];
    matrix_multi((double *)temp13, (double *)temp12, (double *)inv_A_gamma, n_lqr_state, n_lqr_state, n_lqr_state);

    /* H_hat_last = 2*gamma*inv(A_gamma_T + H*inv_A_gamma*G)*H*inv_A_gamma */
    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            sda->H_hat_last[j][k] = 2.0 * sda->gamma * temp13[j][k];
        }
    }
    printf("H_hat_last:\n");
    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            printf("%-6.2f ", sda->H_hat_last[j][k]);
        }
        printf("\n");
    }

    int iter = 0;

    while (1) {

        // reduce redundent calculation by pre-calculating repeated terms
        /* H_hat_last * G_hat_last */
        double temp14[n_lqr_state][n_lqr_state];
        matrix_multi((double *)temp14, (double *)sda->H_hat_last, (double *)sda->G_hat_last, n_lqr_state, n_lqr_state, n_lqr_state);

        double temp15[n_lqr_state][n_lqr_state];
        for (int j = 0; j < n_lqr_state; j++) {
            for (int k = 0; k < n_lqr_state; k++) {
                temp15[j][k] = I[j][k] + temp14[j][k];
            }
        }

        /* inv_I_H_G = inv(I + (H_hat_last * G_hat_last)) */
        double inv_I_H_G[n_lqr_state][n_lqr_state];
        inv_matrix(n_lqr_state, temp15, inv_I_H_G);

        // transpose matrix of A_hat_last
        double A_hat_last_T[n_lqr_state][n_lqr_state];
        matrix_transpose(n_lqr_state, n_lqr_state, sda->A_hat_last, A_hat_last_T);

        // update
        /******* update A_hat_new *******/
        /* G_hat_last * H_hat_last */
        double temp16[n_lqr_state][n_lqr_state];
        matrix_multi((double *)temp16, (double *)sda->G_hat_last, (double *)sda->H_hat_last, n_lqr_state, n_lqr_state, n_lqr_state);

        /* I + G_hat_last * H_hat_last */
        double temp17[n_lqr_state][n_lqr_state];
        for (int j = 0; j < n_lqr_state; j++) {
            for (int k = 0; k < n_lqr_state; k++) {
                temp17[j][k] = I[j][k] + temp16[j][k];
            }
        }

        /* inv(I + G_hat_last * H_hat_last) */
        double temp18[n_lqr_state][n_lqr_state];
        inv_matrix(n_lqr_state, temp17, temp18);

        /* A_hat_last * inv(I + G_hat_last * H_hat_last) */
        double temp19[n_lqr_state][n_lqr_state];
        matrix_multi((double *)temp19, (double *)sda->A_hat_last, (double *)temp18, n_lqr_state, n_lqr_state, n_lqr_state);

        /* A_hat_new = A_hat_last * inv(I + G_hat_last * H_hat_last) * A_hat_last */
        matrix_multi((double *)sda->A_hat_new, (double *)temp19, (double *)sda->A_hat_last, n_lqr_state, n_lqr_state, n_lqr_state);
        // printf("A_hat_new:\n");
        // for (int j = 0; j < n_lqr_state; j++) {
        //     for (int k = 0; k < n_lqr_state; k++) {
        //         printf("%-6.2f ", lqr.A_hat_new[j][k]);
        //     }
        //     printf("\n");
        // }

        /******* update G_hat_new *******/
        /* A_hat_last * G_hat_last */
        double temp20[n_lqr_state][n_lqr_state];
        matrix_multi((double *)temp20, (double *)sda->A_hat_last, (double *)sda->G_hat_last, n_lqr_state, n_lqr_state, n_lqr_state);

        /* A_hat_last * G_hat_last * inv_I_H_G */
        double temp21[n_lqr_state][n_lqr_state];
        matrix_multi((double *)temp21, (double *)temp20, (double *)inv_I_H_G, n_lqr_state, n_lqr_state, n_lqr_state);

        /* A_hat_last * G_hat_last * inv_I_H_G * A_hat_last_T */
        double temp22[n_lqr_state][n_lqr_state];
        matrix_multi((double *)temp22, (double *)temp21, (double *)A_hat_last_T, n_lqr_state, n_lqr_state, n_lqr_state);

        /* G_hat_new = G_hat_last + (A_hat_last * G_hat_last * inv_I_H_G * A_hat_last_T) */
        for (int j = 0; j < n_lqr_state; j++) {
            for (int k = 0; k < n_lqr_state; k++) {
                sda->G_hat_new[j][k] = sda->G_hat_last[j][k] + temp22[j][k];
            }
        }

        /******* update H_hat_new *******/
        /* A_hat_last_T * inv_I_H_G */
        double temp23[n_lqr_state][n_lqr_state];
        matrix_multi((double *)temp23, (double *)A_hat_last_T, (double *)inv_I_H_G, n_lqr_state, n_lqr_state, n_lqr_state);

        /* A_hat_last_T * inv_I_H_G * H_hat_last */
        double temp24[n_lqr_state][n_lqr_state];
        matrix_multi((double *)temp24, (double *)temp23, (double *)sda->H_hat_last, n_lqr_state, n_lqr_state, n_lqr_state);

        /* A_hat_last_T * inv_I_H_G * H_hat_last * A_hat_last */
        double temp25[n_lqr_state][n_lqr_state];
        matrix_multi((double *)temp25, (double *)temp24, (double *)sda->A_hat_last, n_lqr_state, n_lqr_state, n_lqr_state);

        /* H_hat_new = H_hat_last + (A_hat_last_T * inv_I_H_G * H_hat_last * A_hat_last) */
        for (int j = 0; j < n_lqr_state; j++) {
            for (int k = 0; k < n_lqr_state; k++) {
                sda->H_hat_new[j][k] = sda->H_hat_last[j][k] + temp25[j][k];
            }
        }

        for (int j = 0; j < 30; j++){
            printf("-");
        }
        printf("\niteration: %d\n", iter);

        // calculate matrix norms
        double norm_H_hat_last;
        double norm_H_hat_now;
        calculate_norm(n_lqr_state, n_lqr_state, sda->H_hat_last, &norm_H_hat_last);
        calculate_norm(n_lqr_state, n_lqr_state, sda->H_hat_new, &norm_H_hat_now);

        // printf("norm of H_hat_last: %.3lf\n", norm_H_hat_last);
        // printf("norm of H_hat_now : %.3lf\n", norm_H_hat_now);

        double difference_norm = fabs(norm_H_hat_now - norm_H_hat_last);
        if (difference_norm > 0.1){
            printf("difference norm   : %.3lf\n", difference_norm);
        } if (difference_norm <= 0.1){
            printf("difference norm   : %.6lf\n", difference_norm);
        }

        // prepare next iteration
        for (int j = 0; j < n_lqr_state; j++) {
            for (int k = 0; k < n_lqr_state; k++) {
                sda->A_hat_last[j][k] = sda->A_hat_new[j][k];
                sda->G_hat_last[j][k] = sda->G_hat_new[j][k];
                sda->H_hat_last[j][k] = sda->H_hat_new[j][k];
            }
        }

        // stop iteration if converged
        if (difference_norm < 1e-8) {
            break;
        }

        iter++; // do until convergence

    }

    // set X <- H_hat_new
    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            sda->X[j][k] = sda->H_hat_new[j][k];
        }
    }
    printf("X:\n");
    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            printf("%-7.2f ", sda->X[j][k]);
        }
        printf("\n");
    }

}

void LQR_init(){
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

    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_out; k++) {
            if (j == 3 && k == 1) {
                lqr.B[j][k] = 1.0 / I_x;
            } else if (j == 4 && k == 2) {
                lqr.B[j][k] = 1.0 / I_y;
            } else if (j == 5 && k == 3) {
                lqr.B[j][k] = 1.0 / I_z;
            } else if (j == 8 && k == 0) {
                lqr.B[j][k] = -1.0 / mass;
            } else {
                lqr.B[j][k] = 0.0;
            }
        }
    }

    printf("B:\n");
    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_out; k++) {
            printf("%5.2f ", lqr.B[j][k]);
        }
        printf("\n");
    }

    // weight matrix R
    for (int j = 0; j < n_lqr_out; j++) {
        for (int k = 0; k < n_lqr_out; k++) {
            if (j == k) {
                lqr.R[j][k] = 1;
            } else {
                lqr.R[j][k] = 0;
            }
        }
    }

    // transpose matrix of control matrix of state equation
    double B_T[n_lqr_out][n_lqr_state];
    matrix_transpose(n_lqr_state, n_lqr_out, lqr.B, B_T);

    double inv_R[n_lqr_out][n_lqr_out]; // compute inverse matrix of weight matrix R
    inv_matrix(n_lqr_out, lqr.R, inv_R);

    // compute control matrix of state equation B multiplied by inverse matrix of weight matrix R
    double B_inv_R[n_lqr_state][n_lqr_out];
    matrix_multi((double *)B_inv_R, (double *)lqr.B, (double *)inv_R, n_lqr_state, n_lqr_out, n_lqr_out);

    /* G is equal to control matrix B multiplied by inverse matrix of weight matrix R multiplied by transpose matrix of control matrix B */
    matrix_multi((double *)lqr.G, (double *)B_inv_R, (double *)B_T, n_lqr_state, n_lqr_out, n_lqr_state);
    printf("G:\n");
    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            printf("%-6.2f ", lqr.G[j][k]);
        }
        printf("\n");
    }

    // weight matrix Q
    lqr.Q[0][0] = 3000;   // yaw angle
    lqr.Q[1][1] = 10;     // x-axis angular velocity
    lqr.Q[2][2] = 10;     // y-axis angular velocity
    lqr.Q[3][3] = 500;    // z-axis angular velocity
    lqr.Q[4][4] = 1000;   // i-axis velocity represented in body frame
    lqr.Q[5][5] = 1000;   // j-axis velocity represented in body frame
    lqr.Q[6][6] = 1000;   // k-axis velocity represented in body frame
    lqr.Q[7][7] = 5000;   // x-axis position
    lqr.Q[8][8] = 5000;   // y-axis position
    lqr.Q[9][9] = 5000;   // z-axis position

    printf("Q:\n");
    for (int j = 0; j < n_lqr_observation; j++) {
        for (int k = 0; k < n_lqr_observation; k++) {
            printf("%6.0f ", lqr.Q[j][k]);
        }
        printf("\n");
    }

    for (int j = 0; j < n_lqr_observation; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            if (j == k - 2) {
                lqr.C[j][k] = 1;
            } else {
                lqr.C[j][k] = 0;
            }
        }
    }

    printf("C:\n");
    for (int j = 0; j < n_lqr_observation; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            printf("%.1f ", lqr.C[j][k]);
        }
        printf("\n");
    }

    // transpose matrix of C
    double C_T[n_lqr_state][n_lqr_observation];
    matrix_transpose(n_lqr_observation, n_lqr_state, lqr.C, C_T);

    // compute transpose matrix of C multiplied by weight matrix Q
    double C_T_Q[n_lqr_state][n_lqr_observation];
    matrix_multi((double *)C_T_Q, (double *)C_T, (double *)lqr.Q, n_lqr_state, n_lqr_observation, n_lqr_observation);

    /* H is equal to transpose of C multiplied by weight matrix Q multiplied by C */
    matrix_multi((double *)lqr.H, (double *)C_T_Q, (double *)lqr.C, n_lqr_state, n_lqr_observation, n_lqr_state);

    printf("H:\n");
    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            printf("%5.0f ", lqr.H[j][k]);
        }
        printf("\n");
    }

    lqr.position_x_desired = 0.0;
    lqr.position_y_desired = 0.0;
    lqr.position_z_desired = 0.016;
    lqr.velocity_x_desired = 0.0;
    lqr.velocity_y_desired = 0.0;
    lqr.flag = 0;
    sda.gamma = 2.4; // parameter in the Cayley transform
}

double LQR_realize(int i){

    // obtain four rotor motors and set them to speed mode
    WbDeviceTag front_right_motor = wb_robot_get_device("m1_motor");
    WbDeviceTag back_right_motor = wb_robot_get_device("m2_motor");
    WbDeviceTag back_left_motor = wb_robot_get_device("m3_motor");
    WbDeviceTag front_left_motor = wb_robot_get_device("m4_motor");
    WbDeviceTag motors[4] = {front_right_motor, back_right_motor, back_left_motor, front_left_motor};

    const double time = wb_robot_get_time(); // in seconds

    // perturbation of control algorithm by keyboard input
    controller.roll_disturbance = 0.0;  // perturbation of roll angle by keyboard input
    controller.pitch_disturbance = 0.0; // perturbation of pitch angle by keyboard input
    controller.yaw_disturbance = 0.0;   // perturbation of yaw angle by keyboard input

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
            lqr.position_z_desired += 0.05;
            printf("desired altitude: %f [m]\n", lqr.position_z_desired);
            break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
            lqr.position_z_desired -= 0.05;
            printf("desired altitude: %f [m]\n", lqr.position_z_desired);
            break;
        }
        key = wb_keyboard_get_key();
    }

    // plan velocity
    lqr.t_off = 1.77;

    if (time >= lqr.t_off){
        lqr.velocity_x_desired = 0.02;
        lqr.velocity_y_desired = 0.02;
        lqr.velocity_z_desired = 0.05;
    } else if (time < lqr.t_off){
        lqr.velocity_x_desired = 0.0;
        lqr.velocity_y_desired = 0.0;
        lqr.velocity_z_desired = 0.0;
    }

    archive.velocity_x_desired[i] = lqr.velocity_x_desired;
    printf("desired x velocity: %lf\n", lqr.velocity_x_desired);

    archive.velocity_y_desired[i] = lqr.velocity_y_desired;
    printf("desired y velocity: %lf\n", lqr.velocity_y_desired);

    archive.velocity_z_desired[i] = lqr.velocity_z_desired;
    printf("desired z velocity: %lf\n", lqr.velocity_z_desired);

    // plan position
    lqr.position_x_desired = lqr.velocity_x_desired * (time - lqr.t_off);
    lqr.position_y_desired = lqr.velocity_y_desired * (time - lqr.t_off);
    lqr.position_z_desired = lqr.velocity_z_desired * (time - lqr.t_off) + 0.015;

    archive.position_x_desired[i] = lqr.position_x_desired;
    printf("desired x position: %lf\n", lqr.position_x_desired);

    archive.position_y_desired[i] = lqr.position_y_desired;
    printf("desired y position: %lf\n", lqr.position_y_desired);

    archive.position_z_desired[i] = lqr.position_z_desired;
    printf("desired z position: %lf\n", lqr.position_z_desired);

    lqr.state_desired[0] = 0;                       // desired roll angle of drone
    lqr.state_desired[1] = 0;                       // desired pitch angle of drone
    lqr.state_desired[2] = 0;                       // desired yaw angle of drone
    lqr.state_desired[3] = 0;                       // desired x-axis angular velocity of drone
    lqr.state_desired[4] = 0;                       // desired y-axis angular velocity of drone
    lqr.state_desired[5] = 0;                       // desired z-axis angular velocity of drone
    lqr.state_desired[6] = lqr.velocity_x_desired;  // desired i-axis velocity of drone represented in body frame
    lqr.state_desired[7] = lqr.velocity_y_desired;  // desired j-axis velocity of drone represented in body frame
    lqr.state_desired[8] = lqr.velocity_z_desired;  // desired k-axis velocity of drone represented in body frame
    lqr.state_desired[9] = lqr.position_x_desired;  // desired x-axis position of drone
    lqr.state_desired[10] = lqr.position_y_desired; // desired y-axis position of drone
    lqr.state_desired[11] = lqr.position_z_desired; // desired z-axis position of drone
    
    get_A(&ekf.estimated_state, &ekf.state1, lqr.A); // get state matrix of state equation
    printf("A:\n");
    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            printf("%-5.2f ", lqr.A[j][k]);
        }
        printf("\n");
    }

    // Structure-preserving Doubling Algorithm
    care_sda(&lqr, &sda);

    double A_T[n_lqr_state][n_lqr_state]; // transpose matrix of A
    matrix_transpose(n_lqr_state, n_lqr_state, lqr.A, A_T);

    /* A_transpose*X */
    double temp26[n_lqr_state][n_lqr_state];
    matrix_multi((double *)temp26, (double *)A_T, (double *)sda.X, n_lqr_state, n_lqr_state, n_lqr_state);

    /* X*A */
    double temp27[n_lqr_state][n_lqr_state];
    matrix_multi((double *)temp27, (double *)sda.X, (double *)lqr.A, n_lqr_state, n_lqr_state, n_lqr_state);

    /* X*G */
    double temp28[n_lqr_state][n_lqr_state];
    matrix_multi((double *)temp28, (double *)sda.X, (double *)lqr.G, n_lqr_state, n_lqr_state, n_lqr_state);

    /* X*G*X */
    double temp29[n_lqr_state][n_lqr_state];
    matrix_multi((double *)temp29, (double *)temp28, (double *)sda.X, n_lqr_state, n_lqr_state, n_lqr_state);

    /* A_transpose*X + X*A - X*G*X + H */
    double temp30[n_lqr_state][n_lqr_state];
    for (int j = 0; j < n_lqr_state; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            temp30[j][k] = temp26[j][k] + temp27[j][k] - temp29[j][k] + lqr.H[j][k];
        }
    }

    double sda_x_norm;
    calculate_norm(n_lqr_state, n_lqr_state, temp30, &sda_x_norm);
    printf("sda_x_norm: %lf\n", sda_x_norm);

    // transpose matrix of control matrix of state equation
    double B_T[n_lqr_out][n_lqr_state];
    matrix_transpose(n_lqr_state, n_lqr_out, lqr.B, B_T);

    double inv_R[n_lqr_out][n_lqr_out]; // compute inverse matrix of weight matrix R
    inv_matrix(n_lqr_out, lqr.R, inv_R);

    /* inv(R) * transpose(B) */
    double temp31[n_lqr_out][n_lqr_state];
    matrix_multi((double *)temp31, (double *)inv_R, (double *)B_T, n_lqr_out, n_lqr_out, n_lqr_state);

    /* K = inv(R) * transpose(B) * X */
    matrix_multi((double *)lqr.K, (double *)temp31, (double *)sda.X, n_lqr_out, n_lqr_state, n_lqr_state);
    printf("LQR K:\n");
    for (int j = 0; j < n_lqr_out; j++) {
        for (int k = 0; k < n_lqr_state; k++) {
            printf("%-7.2f ", lqr.K[j][k]);
        }
        printf("\n");
    }

    // get_lqr_state(&ekf.estimated_state, &ekf.state1, lqr.state);
    get_lqr_state(&ekf.estimated_state, &ekf.state1, lqr.state);
    printf("\nlqr state:\n[");
    printf("%.3lf", lqr.state[0]);
    for (int j = 1; j < n_lqr_state; j++){
        printf(" %.3lf", lqr.state[j]);
    }
    printf("]\n");

    printf("\nlqr desired state:\n[");
    printf("%.3lf", lqr.state_desired[0]);
    for (int j = 1; j < n_lqr_state; j++){
        printf(" %.3lf", lqr.state_desired[j]);
    }
    printf("]\n");
    
    double state_error[n_lqr_state];
    for (int j = 0; j < n_lqr_state; j++) {
        state_error[j] = lqr.state[j] - lqr.state_desired[j];
    }
    printf("\nstate error:\n[");
    printf("%.3lf", state_error[0]);
    for (int j = 1; j < n_lqr_state; j++){
        printf(" %.3lf", state_error[j]);
    }
    printf("]\n");

    for (int j = 0; j < n_lqr_out; j++){
        if (j == 0){
            lqr.control_feedforward[j] = mass * g * cos(ekf.estimated_state.roll) * cos(ekf.estimated_state.pitch);
        } else if (j > 0){
            lqr.control_feedforward[j] = 0;
        }
    }
    
    printf("\nfeedforward control:\n[");
    printf("%.3lf", lqr.control_feedforward[0]);
    for (int j = 1; j < n_lqr_out; j++){
        printf(" %.3lf", lqr.control_feedforward[j]);
    }
    printf("]\n");

    /* K * [x - x0] */
    double temp32[n_lqr_out];
    matrix_multi((double *)temp32, (double *)lqr.K, (double *)state_error, n_lqr_out, n_lqr_state, 1);
    for (int j = 0; j < n_lqr_out; j++){
        lqr.control_feedback[j] = - temp32[j];
    }
    printf("\nfeedback control:\n[");
    printf("%.3lf", lqr.control_feedback[0]);
    for (int j = 1; j < n_lqr_out; j++){
        printf(" %.3lf", lqr.control_feedback[j]);
    }
    printf("]\n");

    for (int j = 0; j < n_lqr_out; j++){
        lqr.control[j] = lqr.control_feedforward[j] + lqr.control_feedback[j];
    }
    printf("\ncontrol:\n[");
    printf("%.3lf", lqr.control[0]);
    for (int j = 1; j < n_lqr_out; j++){
        printf(" %.3lf", lqr.control[j]);
    }
    printf("]\n");

    if (ekf.true_state.position_z > 0.015){

        // difference of altitude of drone from desired altitude
        controller.altitude_difference = CLAMP(lqr.position_z_desired - ekf.estimated_state.position_z + controller.k_vertical_offset, -1.0, 1.0);
        archive.altitude_difference[i] = controller.altitude_difference;
        printf("altitude_difference: %lf\n", controller.altitude_difference);

        // control input of altitude of quadcopter
        controller.control_altitude = pow(controller.altitude_difference, 3.0);
        archive.control_altitude[i] = controller.control_altitude;
        printf("control_altitude: %lf\n", controller.control_altitude);

        controller.control_roll = lqr.control[1];
        archive.control_roll[i] = controller.control_roll;
        printf("control_roll: %lf\n", controller.control_roll);

        // control input of pitch angle of quadcopter
        controller.control_pitch = lqr.control[2];
        archive.control_pitch[i] = controller.control_pitch;
        printf("control_pitch: %lf\n", controller.control_pitch);

        // control input of yaw angle of quadcopter
        controller.control_yaw = lqr.control[3];
        archive.control_yaw[i] = controller.control_yaw;
        printf("control_yaw: %lf\n", controller.control_yaw);

        double c1 = 3.0;
        double c2 = 0.001;
        double c3 = 0.001;
        double c4 = 0.001;

        // input value for front right motor of the quadcopter
        controller.front_right_motor_input = controller.k_vertical_thrust + c1 * controller.control_altitude + c2 * ( + controller.control_roll) + c3 * ( + controller.control_pitch) + c4 * ( + controller.control_yaw);
        archive.front_right_motor_input[i] = controller.front_right_motor_input;
        printf("front_right_motor_input: %.3lf\n", controller.front_right_motor_input);

        // input value for back right motor of the quadcopter
        controller.back_right_motor_input = controller.k_vertical_thrust + c1 * controller.control_altitude + c2 * ( + controller.control_roll) + c3 * ( - controller.control_pitch) + c4 * ( - controller.control_yaw);
        archive.back_right_motor_input[i] = controller.back_right_motor_input;
        printf("back_right_motor_input: %.3lf\n", controller.back_right_motor_input);

        // input value for back left motor of the quadcopter
        controller.back_left_motor_input = controller.k_vertical_thrust + c1 * controller.control_altitude + c2 * ( - controller.control_roll) + c3 * ( - controller.control_pitch) + c4 * ( + controller.control_yaw);
        archive.back_left_motor_input[i] = controller.back_left_motor_input;
        printf("back_left_motor_input: %.3lf\n", controller.back_left_motor_input);

        // input value for front left motor of the quadcopter
        controller.front_left_motor_input = controller.k_vertical_thrust + c1 * controller.control_altitude + c2 * ( - controller.control_roll) + c3 * ( + controller.control_pitch) + c4 * ( - controller.control_yaw);
        archive.front_left_motor_input[i] = controller.front_left_motor_input;
        printf("front_left_motor_input: %.3lf\n", controller.front_left_motor_input);

    } else if (ekf.true_state.position_z <= 0.015){

        // input value for front right motor of the quadcopter
        controller.front_right_motor_input = 57.0;
        archive.front_right_motor_input[i] = controller.front_right_motor_input;
        printf("front_right_motor_input: %.3lf\n", controller.front_right_motor_input);

        // input value for back right motor of the quadcopter
        controller.back_right_motor_input = 57.0;
        archive.back_right_motor_input[i] = controller.back_right_motor_input;
        printf("back_right_motor_input: %.3lf\n", controller.back_right_motor_input);

        // input value for back left motor of the quadcopter
        controller.back_left_motor_input = 57.0;
        archive.back_left_motor_input[i] = controller.back_left_motor_input;
        printf("back_left_motor_input: %.3lf\n", controller.back_left_motor_input);

        // input value for front left motor of the quadcopter
        controller.front_left_motor_input = 57.0;
        archive.front_left_motor_input[i] = controller.front_left_motor_input;
        printf("front_left_motor_input: %.3lf\n", controller.front_left_motor_input);

    }
    wb_motor_set_velocity(front_right_motor, -controller.front_right_motor_input);
    wb_motor_set_velocity(back_right_motor, controller.back_right_motor_input);
    wb_motor_set_velocity(back_left_motor, -controller.back_left_motor_input);
    wb_motor_set_velocity(front_left_motor, controller.front_left_motor_input);
}
