#include <stdio.h>
#include <stdlib.h>
#include "savefile.h"

_archive archive;

void saveArchiveToTxt(double *archive1, int size, const char *filename) {
    FILE *file = fopen(filename, "w+");
    if (file == NULL) {
        perror("Failed to open file");
        exit(1);
    }
    else {
        for (int j = 0; j < size; j++) {
            fprintf(file, "%lf\n", archive1[j]);
        }
        fclose(file);
        printf("Saved to file %s\n", filename);
    }
}

void saveArchive() {
    saveArchiveToTxt(archive.position_x, ARRAY_SIZE, "../../../report/position_x.txt");
    saveArchiveToTxt(archive.position_y, ARRAY_SIZE, "../../../report/position_y.txt");
    saveArchiveToTxt(archive.position_z, ARRAY_SIZE, "../../../report/position_z.txt");
    saveArchiveToTxt(archive.velocity_x, ARRAY_SIZE, "../../../report/velocity_x.txt");
    saveArchiveToTxt(archive.velocity_y, ARRAY_SIZE, "../../../report/velocity_y.txt");
    saveArchiveToTxt(archive.velocity_z, ARRAY_SIZE, "../../../report/velocity_z.txt");
    saveArchiveToTxt(archive.roll, ARRAY_SIZE, "../../../report/roll.txt");
    saveArchiveToTxt(archive.pitch, ARRAY_SIZE, "../../../report/pitch.txt");
    saveArchiveToTxt(archive.yaw, ARRAY_SIZE, "../../../report/yaw.txt");
    saveArchiveToTxt(archive.velocity_i, ARRAY_SIZE, "../../../report/velocity_i.txt");
    saveArchiveToTxt(archive.velocity_j, ARRAY_SIZE, "../../../report/velocity_j.txt");
    saveArchiveToTxt(archive.velocity_k, ARRAY_SIZE, "../../../report/velocity_k.txt");
    saveArchiveToTxt(archive.acceleration_x, ARRAY_SIZE, "../../../report/acceleration_x.txt");
    saveArchiveToTxt(archive.acceleration_y, ARRAY_SIZE, "../../../report/acceleration_y.txt");
    saveArchiveToTxt(archive.acceleration_z, ARRAY_SIZE, "../../../report/acceleration_z.txt");
    saveArchiveToTxt(archive.roll_angular_velocity, ARRAY_SIZE, "../../../report/roll_angular_velocity.txt");
    saveArchiveToTxt(archive.pitch_angular_velocity, ARRAY_SIZE, "../../../report/pitch_angular_velocity.txt");
    saveArchiveToTxt(archive.yaw_angular_velocity, ARRAY_SIZE, "../../../report/yaw_angular_velocity.txt");
    saveArchiveToTxt(archive.angular_velocity_x, ARRAY_SIZE, "../../../report/angular_velocity_x.txt");
    saveArchiveToTxt(archive.angular_velocity_y, ARRAY_SIZE, "../../../report/angular_velocity_y.txt");
    saveArchiveToTxt(archive.angular_velocity_z, ARRAY_SIZE, "../../../report/angular_velocity_z.txt");
    saveArchiveToTxt(archive.control_roll, ARRAY_SIZE, "../../../report/control_roll.txt");
    saveArchiveToTxt(archive.control_pitch, ARRAY_SIZE, "../../../report/control_pitch.txt");
    saveArchiveToTxt(archive.control_yaw, ARRAY_SIZE, "../../../report/control_yaw.txt");
    saveArchiveToTxt(archive.control_altitude, ARRAY_SIZE, "../../../report/control_altitude.txt");
    // saveArchiveToTxt(archive.altitude_difference, ARRAY_SIZE, "../../../report/altitude_difference.txt");
    saveArchiveToTxt(archive.front_left_motor_input, ARRAY_SIZE, "../../../report/front_left_motor_input.txt");
    saveArchiveToTxt(archive.front_right_motor_input, ARRAY_SIZE, "../../../report/front_right_motor_input.txt");
    saveArchiveToTxt(archive.back_left_motor_input, ARRAY_SIZE, "../../../report/back_left_motor_input.txt");
    saveArchiveToTxt(archive.back_right_motor_input, ARRAY_SIZE, "../../../report/back_right_motor_input.txt");
}