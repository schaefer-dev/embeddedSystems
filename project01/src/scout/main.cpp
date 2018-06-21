
#include <OrangutanTime.h>
#include <OrangutanSerial.h>
#include "main.h"
#include <math.h>
#include "../collector/Coordinates.h"
#include "../scout/ScoutMonitor.h"


CoordinateQueue *coordinateQueue;
ScoutState *scoutState;
bool spiEnabled = true;

int main() {
    /* SETUP */
#ifdef SCOUT_MONITOR
    ScoutMonitor::verifyState();
    ScoutMonitor::logPingScout();
    ScoutMonitor::logPingScout();
    ScoutMonitor::logPingScout();
    ScoutMonitor::logPongScout();
    ScoutMonitor::logPingScout();
    ScoutMonitor::logSendHarvest();     // should give good trace alarm
    ScoutMonitor::emptyBuffer();
#endif

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

    delay(200);

    if (spiEnabled) {
        ScoutSPI::SPIMasterInit();
        delay(50);
        ScoutSPI::initializeRFModule();
        delay(100);
    }
    

    while (1) {

        //printPhotosensorReadings();

        delay(150);

        //ScoutSPI::queryRFModule();

        ScoutSerial::serialWrite("REGISTER CHECk START:\n", 23);
        ScoutSPI::debug_RFModule();
        ScoutSerial::serialWrite("REGISTER CHECk END:\n", 20);
        delay(1000);
    }

}


void driveToSerialInput(){
    readNewDestinations();
    if (driveToDestination()) {
        performRotation();
    }
    scoutState->updateRoboterPositionAndAngles();
}


/* write Readings of all Photosensors to DEV port serial */
void printPhotosensorReadings(){
    int adcout11, adcout0, adcout1, adcout2, adcout3;

    adcout11 = ScoutSPI::readADC(0);
    ScoutSPI::ADCConversionWait();

    adcout0 = ScoutSPI::readADC(1);
    ScoutSPI::ADCConversionWait();

    adcout1 = ScoutSPI::readADC(2);
    ScoutSPI::ADCConversionWait();

    adcout2 = ScoutSPI::readADC(3);
    ScoutSPI::ADCConversionWait();

    adcout3 = ScoutSPI::readADC(11);
    ScoutSPI::ADCConversionWait();

    ScoutSerial::serialWrite("LIGHT: front=", 13);
    ScoutSerial::serialWrite8Bit(adcout0);

    ScoutSerial::serialWrite(" right=", 7);
    ScoutSerial::serialWrite8Bit(adcout1);

    ScoutSerial::serialWrite(" back=", 7);
    ScoutSerial::serialWrite8Bit(adcout2);

    ScoutSerial::serialWrite(" left=", 6);
    ScoutSerial::serialWrite8Bit(adcout3);

    ScoutSerial::serialWrite("\n", 1);
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