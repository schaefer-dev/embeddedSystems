//
// Created by Daniel Schäfer on 05.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_MAIN_H
#define EMBEDDEDSYSTEMS18_MAIN_H


/* debug define to enable serial output/inputs to save program memory */
#define DEBUG

#define ROBOT_SCOUT

void setup();
void loop();
bool driveToDestination();
bool readNewDestinations(int[]);
void performRotation();
void performStraightDrive(int cmLength);
void huntObject();
void generateBrightnessLevels();
void setTimer(int);
void driveToSerialInput();
void homing();
void checkForNewRFMessage();


#endif //EMBEDDEDSYSTEMS18_MAIN_H