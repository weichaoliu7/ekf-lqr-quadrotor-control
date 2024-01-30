#ifndef LQR
#define LQR_H

#include "ekf.h"

#define PI 3.14159
#define n_lqr_state 12       // number of state variables of LQR controller
#define n_lqr_out 4          // number of output variables of LQR controller
#define n_lqr_observation 10 // number of observation variables of LQR controller

static double t0 = 0.0;     // start time
static double t1 = 15.0;    // end time
static double I_x = 0.022;  // moment of inertia of drone around x-axis
static double I_y = 0.022;  // moment of inertia of drone around y-axis
static double I_z = 0.03;   // moment of inertia of drone around z-axis
static double mass = 0.895; // mass of drone

typedef struct{
    double position_x_desired;      // desired x-axis position of drone
    double position_y_desired;      // desired y-axis position of drone
    double position_z_desired;      // desired z-axis position of drone
    double velocity_x_desired;      // desired x-axis velocity of drone
    double velocity_y_desired;      // desired y-axis velocity of drone
    double velocity_z_desired;      // desired z-axis velocity of drone
    double radius;                  // radius of circular trajectory
    double circum_angular_velocity; // circumferential angular velocity
    double t_off;                   // ground leaving time
    int flag;
    int flag1;
    double A[n_lqr_state][n_lqr_state];       // state matrix for state equation
    double B[n_lqr_state][n_lqr_out];         // control matrix for state equation
    double C[n_lqr_observation][n_lqr_state]; // construct matrix (roll and pitch are mask out)
    double R[n_lqr_out][n_lqr_out];           // weight matrix about control
    double G[n_lqr_state][n_lqr_state];
    double Q[n_lqr_observation][n_lqr_observation]; // weight matrix about observation
    double H[n_lqr_state][n_lqr_state];
    double state[n_lqr_state];
    double state_desired[n_lqr_state];
    double K[n_lqr_out][n_lqr_state];
    double control[n_lqr_out];
    double control_feedforward[n_lqr_out];
    double control_feedback[n_lqr_out];
} _lqr;

typedef struct{
    double gamma; // parameter in the Cayley transform
    double A_gamma[n_lqr_state][n_lqr_state];
    double A_hat_last[n_lqr_state][n_lqr_state];
    double G_hat_last[n_lqr_state][n_lqr_state];
    double H_hat_last[n_lqr_state][n_lqr_state];
    double A_hat_new[n_lqr_state][n_lqr_state];
    double G_hat_new[n_lqr_state][n_lqr_state];
    double H_hat_new[n_lqr_state][n_lqr_state];
    double X[n_lqr_state][n_lqr_state];
} _sda;

void get_A(const state_variable *state, const system_state *state1, double A[n_lqr_state][n_lqr_state]);

void LQR_init();

double LQR_realize(int i);

#endif // ARCHIVE_H
