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
    saveArchiveToTxt(archive.roll_angular_velocity, ARRAY_SIZE, "../../../report/roll_angular_velocity.txt");
}
