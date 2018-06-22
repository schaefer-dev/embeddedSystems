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

#define ROBOT_SCOUT

int main();
bool driveToDestination();
bool readNewDestinations(int[]);
void performRotation(int degrees);
void debug_printPhotosensorReadings();
void performStraightDrive(int cmLength);
void driveToSerialInput();
void photophobicScout();
void checkForNewRFMessage();

#endif //EMBEDDEDSYSTEMS18_SCOUT_MAIN_H

