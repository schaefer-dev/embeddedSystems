//
// Created by Daniel Schäfer on 06.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_SCOUT_MAIN_H
#define EMBEDDEDSYSTEMS18_SCOUT_MAIN_H

#include <stdlib.h>
#include "ScoutState.h"
#include "ScoutSPI.h"
#include "ScoutSerial.h"

/* debug define to enable serial output/inputs to save program memory */
#define DEBUG

int main();
bool driveToDestination();
void readNewDestinations();
void performRotation();
void performStraightDrive(int cmLength);

#endif //EMBEDDEDSYSTEMS18_SCOUT_MAIN_H

