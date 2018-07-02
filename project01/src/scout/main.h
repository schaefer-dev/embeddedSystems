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

#define ARENA_SIZE_X 140
//#define ROBOT_SIMULATOR

void initialize();
int main();
bool driveToDestination();
bool readNewDestinations(int[]);
void performRotation(int degrees);
void debug_printPhotosensorReadings();
void performStraightDrive(int cmLength);
void driveToSerialInput();
void photophobicScout();
void checkForNewRFMessage();
void debug_sendPingToCollector();
void homing();
void checkForLines();

void receivePosUpdate(unsigned int angle, unsigned int x, unsigned int y);

#endif //EMBEDDEDSYSTEMS18_SCOUT_MAIN_H

