//
// Created by Daniel Schäfer on 06.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_MAIN_H
#define EMBEDDEDSYSTEMS18_MAIN_H

#include <stdlib.h>

/* debug define to enable serial output/inputs to save program memory */
#define DEBUG

int main();
bool driveToDestination();
void readNewDestinations();
void performRotation();
void performStraightDrive(int cmLength);

#endif //EMBEDDEDSYSTEMS18_MAIN_H

