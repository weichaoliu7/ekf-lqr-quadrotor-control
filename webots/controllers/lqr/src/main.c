#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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

/* reference: [1] Raja M M. Extended Kalman Filter and LQR controller design for quadrotor UAVs[J]. 2017.
[2] https://mp.weixin.qq.com/s/_nIWlXXyhAKhyTltez72HQ
[3] 黄小平, 王岩. 卡尔曼滤波原理及应用: MATLAB 仿真[M]. 电子工业出版社, 2015.
[4] https://github.com/shengwen-tw/quadrotor-lqr-simulator
[5] Chu E K W, Fan H Y, Lin W W. A structure-preserving doubling algorithm for continuous-time algebraic Riccati equations[J].
 Linear algebra and its applications, 2005, 396: 55-80.*/

extern _archive archive;
extern _ekf ekf;
extern _controller controller;
extern _pid pid;
extern _lqr lqr;
extern _sda sda;
extern double t0;
extern double t1;

int main(int argc, char **argv){

    EKF_init(); // initialize EKF parameter
    // PID_init(); // initialize PID controller parameter
    LQR_init(); // initialize LQR controller parameter
    // PLANT_init();   // initialize plant parameter

    int i = 0;
    while (wb_robot_step(TIME_STEP) != -1){
        for (int j = 0; j < 60; j++){
            printf("*");
        }
        printf("\n");
        double time = i * TIME_STEP / 1000.0 + t0;
        printf("time at step %d: %f\n", i, time);

        if (time > t1){
            break;
        }

        EKF_realize(i);
        // PID_realize(i);
        LQR_realize(i);
        // PLANT_realize(i);
        i++;
    }

    // saveArchive();

    wb_robot_cleanup();

    return 0;
}
