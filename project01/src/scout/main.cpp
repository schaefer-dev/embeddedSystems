
#include <OrangutanTime.h>
#include <OrangutanSerial.h>
#include "main.h"
#include <math.h>
#include "../collector/Coordinates.h"


CoordinateQueue *coordinateQueue;
ScoutState *scoutState;
bool spiEnabled = true;

int main() {
    /* SETUP */

    /* initialization of Data structures */
    scoutState = new ScoutState();
    coordinateQueue = new CoordinateQueue();

    // initialize differential updateRoboterPositionAndAngles
    scoutState->setSpeeds(0, 0);
    scoutState->resetDifferentialDrive(0, 0, 0);

#ifdef DEBUG
    // initialize serial connection
    ScoutSerial::initScoutSerial();
    OrangutanSerial::setBaudRate(9600);
    ScoutSerial::serialWrite("--- Start Serial Monitor ---\n", 29);
#endif

   scoutState->lastDiffDriveCall = millis();


    /* DEBUG: Testing insertions of coordinates */
    //coordinateQueue->append(30,0);

    delay(200);

    char adcdata;

    if (spiEnabled) {
        ScoutSPI::SPIMasterInit();
        delay(500);
    }

    int adcIndex = 0;
    int adcout11[10];
    int adcout0[10];
    int adcout1[10];
    int adcout2[10];
    int adcout3[10];

    while (1) {

        int adcout11Average = 0;
        int adcout0Average = 0;
        int adcout1Average = 0;
        int adcout2Average = 0;
        int adcout3Average = 0;


        /* DEBUG continious reading test without deselect
         *
        adcout0 = ScoutSPI::readADCContinous(0);
        ScoutSerial::serialWriteInt(adcout0);

        adcout1 = ScoutSPI::readADCContinous(1);
        ScoutSerial::serialWriteInt(adcout1);

        adcout2 = ScoutSPI::readADCContinous(2);
        ScoutSerial::serialWriteInt(adcout2);

        adcout3 = ScoutSPI::readADCContinous(3);
        ScoutSerial::serialWriteInt(adcout3);

        adcout11 = ScoutSPI::readADCContinous(11);
        ScoutSerial::serialWriteInt(adcout11);

        delay(100);
         */



        /* WORKING: debug: testing to read in interrupt
        ScoutSPI::readADCInInterrupt(0);
        delay(10);
        ScoutSPI::readADCInInterrupt(0);
        delay(10);
        ScoutSPI::readADCInInterrupt(0);
        delay(10);
         */


        adcout11[adcIndex] = ScoutSPI::readADC(0);

        delay(1);

        adcout0[adcIndex] = ScoutSPI::readADC(1);

        delay(1);

        adcout1[adcIndex] = ScoutSPI::readADC(2);

        delay(1);

        adcout2[adcIndex] = ScoutSPI::readADC(3);

        delay(1);

        adcout3[adcIndex] = ScoutSPI::readADC(11);

        delay(1);

        for (int i = 0; i < 10; i++){
            adcout11Average += adcout11[i];
            adcout0Average += adcout0[i];
            adcout1Average += adcout1[i];
            adcout2Average += adcout2[i];
            adcout3Average += adcout3[i];
        }

        adcout0Average = adcout0Average / 10;
        adcout1Average = adcout1Average / 10;
        adcout2Average = adcout2Average / 10;
        adcout3Average = adcout3Average / 10;


        ScoutSerial::serialWrite("LIGHT: left=", 12);
        //ScoutSerial::serialWrite8Bit(adcout0Average);
        ScoutSerial::serialWrite8Bit(adcout0[adcIndex]);
        ScoutSerial::serialWrite(" front=", 7);
        //ScoutSerial::serialWrite8Bit(adcout1Average);
        ScoutSerial::serialWrite8Bit(adcout1[adcIndex]);
        ScoutSerial::serialWrite(" right=", 7);
        //ScoutSerial::serialWrite8Bit(adcout2Average);
        ScoutSerial::serialWrite8Bit(adcout2[adcIndex]);
        ScoutSerial::serialWrite(" back=", 6);
        //ScoutSerial::serialWrite8Bit(adcout3Average);
        ScoutSerial::serialWrite8Bit(adcout3[adcIndex]);

        ScoutSerial::serialWrite("\n", 1);

        delay(150);

        adcIndex = (adcIndex + 1) % 10;



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

    bool newCoordinates = ScoutSerial::readCoordinates(coordinates);

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