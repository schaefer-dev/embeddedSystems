
#include <OrangutanTime.h>
#include <OrangutanSerial.h>
#include "main.h"
#include <math.h>
#include "../collector/Coordinates.h"


CoordinateQueue *coordinateQueue;
ScoutState *scoutState;
ScoutSerial *scoutSerial;
SPIMaster *scoutSPI;
bool spiEnabled = true;

int main() {
    /* SETUP */

    /* initialization of Data structures */
    scoutState = new ScoutState();
    coordinateQueue = new CoordinateQueue();
    scoutSPI = new SPIMaster();

    // initialize differential updateRoboterPositionAndAngles
    scoutState->setSpeeds(0, 0);
    scoutState->resetDifferentialDrive(0, 0, 0);

#ifdef DEBUG
    // initialize serial connection
    scoutSerial = new ScoutSerial();
    OrangutanSerial::setBaudRate(9600);
    scoutSerial->serialWrite("--- Start Serial Monitor ---\n", 29);
#endif

   scoutState->lastDiffDriveCall = millis();


    /* DEBUG: Testing insertions of coordinates */
    //coordinateQueue->append(30,0);

    delay(200);

    char adcdata;

    if (spiEnabled)
        scoutSPI->SPIMasterInit();

    while (1) {

        /* IMPORTANT: SPI CODE DISABLED */
        if (PORTB & (1<<PB4)){
            scoutSerial->serialWrite("high\n",5);
        } else {
            scoutSerial->serialWrite("low\n",4);
        }
        delay(100);




        /* IMPORTANT Roboter driving code ENABLED */
        if (!spiEnabled) {

            readNewDestinations();
            if (driveToDestination()) {
                performRotation();
            }
            scoutState->updateRoboterPositionAndAngles();
        }

    }

}


/**
 * When the current destination is reached, calls @readNewDestinations.
 * Drives to the current destination.
 * @return True iff the last destination is reached.
 */
bool driveToDestination() {
    if (scoutState->destinationReached) {
        struct CoordinateQueue::CoordinateNode *node = coordinateQueue->pop(scoutState->currentX, scoutState->currentY);
        if (node == nullptr) {
            scoutState->setSpeeds(0, 0);
            return false;
        }

        scoutState->destinationX = node->x;
        scoutState->destinationY = node->y;
        scoutState->destinationReached = false;
    }
    return scoutState->navigateToDestination();
}

/**
 * Reads a new destination from the serial stream and adds it to the queue.
 * New destination must be given as to integers separated by some non-numeric character
 */
void readNewDestinations() {
    // read new destination entry
/* IMPORTANT:
 * Reading serial is what blows memory up, around 30% - should be disabled once we get close to 100% */

    int coordinates[2];

    bool newCoordinates = scoutSerial->readCoordinates(coordinates);

    if (!newCoordinates) {
        return;
    }

    int xDestination = coordinates[0];
    int yDestination = coordinates[1];

    coordinateQueue->append(xDestination, yDestination);
}

/**
 * Performs the celebration rotation
 */
void performRotation() {
    float startAngle = scoutState->currentAngle;
    bool loopCondition = true;

    while (loopCondition) {

        // turn right
        scoutState->setSpeeds(scoutState->turningSpeed, -scoutState->turningSpeed);
        scoutState->updateRoboterPositionAndAngles();

        /* rotation completed condition */
        if (scoutState->currentAngle > startAngle + 2 * M_PI ||
            scoutState->currentAngle < startAngle - 2 * M_PI) {
            loopCondition = false;
#ifdef DEBUG
            //serial_send("One rotation performed!\n", 24);
#endif
            scoutState->setSpeeds(0, 0);

        }
    }
}

void performStraightDrive(int cmLength) {
    float startX = scoutState->currentX;
    float targetX = startX + cmLength;
    bool loopCondition = true;

    while (loopCondition) {

        // drive straight
        scoutState->setSpeeds(scoutState->forwardSpeed, scoutState->forwardSpeed);

        delay(10);

        scoutState->updateRoboterPositionAndAngles();

        /* rotation completed condition */
        if (scoutState->currentX > targetX) {
            loopCondition = false;
#ifdef DEBUG
            //serial_send("Driving performed!\n", 19);
#endif
            scoutState->setSpeeds(0, 0);

        } else {
#ifdef DEBUG
            //Serial1.println(scoutState->currentX);
#endif
        }
    }
}


void operator delete(void* ptr) { free(ptr); }

void *operator new(size_t size) {
    return malloc(size);
}

void *operator new[](size_t size) {
    return malloc(size);
}