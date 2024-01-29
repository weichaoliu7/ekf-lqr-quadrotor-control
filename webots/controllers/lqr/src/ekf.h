#ifndef EKF_H
#define EKF_H

// global variables declaration
#define TIME_STEP 10
#define n_state 15       // number of state variable
#define n_measurement 10 // number of state measurement variable
static double g = 9.81;       // gravitational acceleration
static double Ts = 0.010;     // sampling period

typedef struct{
    double angular_velocity_x;     // x-axis angular velocity of drone
    double angular_velocity_y;     // y-axis angular velocity of drone
    double angular_velocity_z;     // z-axis angular velocity of drone
    double acceleration_x;         // x-axis acceleration of drone
    double acceleration_y;         // y-axis acceleration of drone
    double acceleration_z;         // z-axis acceleration of drone
    double velocity_x;             // x-axis velocity of drone
    double velocity_y;             // y-axis velocity of drone
    double velocity_z;             // z-axis velocity of drone
    double roll_angular_velocity;  // roll angular velocity of drone
    double pitch_angular_velocity; // pitch angular velocity of drone
    double yaw_angular_velocity;   // yaw angular velocity of drone
} system_state;

// define state variable and measurement variable
typedef struct{
    double position_x;              // x-axis position of drone
    double position_y;              // y-axis position of drone
    double position_z;              // z-axis position of drone
    double velocity_i;              // i-axis velocity of drone represented in body frame
    double velocity_j;              // j-axis velocity of drone represented in body frame
    double velocity_k;              // k-axis velocity of drone represented in body frame
    double roll;                    // roll angle of drone
    double pitch;                   // pitch angle of drone
    double yaw;                     // yaw angle of drone
    double bias_angular_velocity_x; // x-axis angular velocity bias of drone
    double bias_angular_velocity_y; // y-axis angular velocity bias of drone
    double bias_angular_velocity_z; // z-axis angular velocity bias of drone
    double bias_acceleration_x;     // x-axis acceleration bias of drone
    double bias_acceleration_y;     // y-axis acceleration bias of drone
    double bias_acceleration_z;     // z-axis acceleration bias of drone
} state_variable;

typedef struct{
    double measurement_angular_velocity_x; // x-axis measurement angular velocity bias of drone
    double measurement_angular_velocity_y; // y-axis measurement angular velocity bias of drone
    double measurement_angular_velocity_z; // z-axis measurement angular velocity bias of drone
    double measurement_acceleration_x;     // x-axis measurement acceleration bias of drone
    double measurement_acceleration_y;     // y-axis measurement acceleration bias of drone
    double measurement_acceleration_z;     // z-axis measurement acceleration bias of drone
    double measurement_position_x;         // x-axis measurement position of drone
    double measurement_position_y;         // y-axis measurement position of drone
    double measurement_position_z;         // z-axis measurement position of drone
    double measurement_yaw;                // measurement yaw angle of drone
} measurement_variable;

typedef struct{
    state_variable true_state;      // true state variable
    state_variable estimated_state; // initialize estimated state variable
    state_variable predicted_state; // predicted state variable
    state_variable updated_state;   // updated state variable
    state_variable estimated_error; // estimated error variable
    system_state state1;
    measurement_variable measurement;              // observation variable
    measurement_variable measurement_predicted;    // predicted observation variable
    double covariance[n_state][n_state];           // covariance matrix P
    double predicted_covariance[n_state][n_state]; // predicted covariance matrix
    double updated_covariance[n_state][n_state];   // updated covariance matrix
    double Q[n_state][n_state];                    // process noise W covariance matrix Q
    double R[n_measurement][n_measurement];        // gaussian white noise V covariance matrix R
    double K[n_state][n_measurement];              // kalman gain
    double jacobian_F[n_state][n_state];           // jacobian matrice of nonlinear function f on state variable x, Phi
    double jacobian_H[n_measurement][n_state];     // observation matrix H, i.e., jacobian matrix of observation function h for state variable x
    double last_position_x;                        // x-axis position of drone at last sampling moment
    double last_position_y;                        // y-axis position of drone at last sampling moment
    double last_position_z;                        // z-axis position of drone at last sampling moment
    double last_velocity_x;                        // x-axis velocity of drone at last sampling moment
    double last_velocity_y;                        // y-axis velocity of drone at last sampling moment
    double last_velocity_z;                        // z-axis velocity of drone at last sampling moment
} _ekf;

// state transition function
void state_transition_function(const state_variable *state, const double *control, const system_state *state1, state_variable *new_state);

// observation function
void observation_function(const state_variable *state, const system_state *state1, measurement_variable *observation);

// inertial frame converted to body frame
void InertialtoBody(const state_variable *state, const system_state *state1, state_variable *new_state, system_state *new_state1);

void get_jacobian_F(const state_variable *state, const system_state *state1, double J[n_state][n_state]);

void EKF_init();

// extended kalman filter
void EKF_realize(int i);

#endif // EKF_H
