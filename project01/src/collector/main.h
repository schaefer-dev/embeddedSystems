//
// Created by Daniel Schäfer on 05.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_MAIN_H
#define EMBEDDEDSYSTEMS18_MAIN_H


/* debug define to enable serial output/inputs to save program memory */
#define DEBUG

#define ARENA_SIZE_X 140

#define ROBOT_SIMULATOR

void setup();
void loop();
bool driveToDestination();
bool readNewDestinations(int[]);
void performRotation(int degrees);
void performStraightDrive(int cmLength);
void huntObject();
void generateBrightnessLevels();
void setTimer(int);
void driveToSerialInput();
void homing();
void checkForNewRFMessage();

void receivePosUpdate(unsigned int angle, unsigned int x, unsigned int y);

#endif //EMBEDDEDSYSTEMS18_MAIN_H