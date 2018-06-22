
#include <OrangutanTime.h>
#include <OrangutanSerial.h>
#include "main.h"
#include <math.h>
#include "../utils/Coordinates.h"
#include "../scout/ScoutMonitor.h"
#include "ScoutRF.h"


#define photophobicWaitThreshold 50
#define photophobicDanceThreshold 100
#define PHOTOPHOBIC_ROTATION 60


CoordinateQueue *coordinateQueue;
ScoutState *scoutState;
bool spiEnabled = true;
int statusRF = 0;

int main() {
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
    //delay(600);
    //ScoutMonitor::verifyState();
#endif

    scoutState->lastDiffDriveCall = millis();

    delay(200);

    if (spiEnabled) {
        ScoutSPI::SPIMasterInit();
        delay(50);
        ScoutRF::initializeRFModule();
        delay(100);

        /* RF DEBUGGING
        ScoutSerial::serialWrite("REGISTER CHECk START:\n", 23);
        ScoutSPI::debug_RFModule();
        ScoutSerial::serialWrite("REGISTER CHECk END:\n", 20);
        */
    }


    while (1) {

        /* photophobic mode
        debug_printPhotosensorReadings();
        photophobicScout();
        delay(100);
        */

        if (spiEnabled) {

            checkForNewRFMessage();

            delay(1000);
        }
    }

}

void checkForNewRFMessage(){
    statusRF = ScoutRF::queryRFModule();
    char messageReceived = statusRF & (1 << 6);

    if (messageReceived){
        /* case for Message arrived */
        int answerArray[1];
        ScoutRF::getCommandAnswer(answerArray, 1, RF_COMMAND_R_RX_PL_WID);

        /* read message from pipe */
        int payloadArray[answerArray[0]];
        ScoutRF::getCommandAnswer(payloadArray, answerArray[0], RF_COMMAND_R_RX_PAYLOAD);

        ScoutSerial::serialWrite("Message: ", 9);

        for (int i = 0; i < answerArray[0]; i++){
            ScoutSerial::serialWrite8BitHex(payloadArray[i]);
        }
        ScoutSerial::serialWrite("\n",1);

        /* clear status register */
        ScoutRF::writeRegister(RF_REGISTER_STATUS, 64);

        switch(payloadArray[0]){
            case 0x50:
                /* PING case */
                ScoutRF::sendPongToReferee(payloadArray[1] + payloadArray[2]);
                break;
            case 0x60:
                /* POS update case */
                break;
            case 0x70:
                /* MESSAGE case */
                break;
            default:
                ScoutSerial::serialWrite("Illegal Message Identifer\n",26);
        }

    }
}

void driveToSerialInput() {
    int destination[2];
    readNewDestinations(destination);
    coordinateQueue->append(destination[0], destination[1]);

    if (driveToDestination()) {
        performRotation(360);
    }
    scoutState->updateRoboterPositionAndAngles();
}


/* run photophobic Scout Controller as explained in Milestone 5
 * If all photosensors less then 100, dance in place, if all sensors in range of
 * defined threshold stand still, otherwise follow the shadow */
void photophobicScout() {
    scoutState->updatePhotoSensorReadings();

    if (scoutState->photoSensorLeft < photophobicDanceThreshold &&
        scoutState->photoSensorBack < photophobicDanceThreshold &&
        scoutState->photoSensorRight < photophobicDanceThreshold &&
        scoutState->photoSensorFront < photophobicDanceThreshold){
        /* start dancing */
        while (true){
            performRotation(180);
            performRotation(-180);
        }
    }

    /* calculate abs of photosensor readings */
    int diffFrontLeft = scoutState->photoSensorFront - scoutState->photoSensorLeft;
    diffFrontLeft = (diffFrontLeft < 0) ? -diffFrontLeft : diffFrontLeft;

    int diffFrontRight = scoutState->photoSensorFront - scoutState->photoSensorRight;
    diffFrontRight = (diffFrontRight < 0) ? -diffFrontRight : diffFrontRight;

    int diffFrontBack = scoutState->photoSensorFront - scoutState->photoSensorBack;
    diffFrontBack = (diffFrontBack < 0) ? -diffFrontBack : diffFrontBack;

    if (diffFrontBack < photophobicWaitThreshold && diffFrontLeft < photophobicWaitThreshold &&
        diffFrontRight < photophobicWaitThreshold) {
        /* stand still */
        ScoutSerial::serialWrite("Stant still\n", 12);

        scoutState->setSpeeds(0,0);
    } else {
        if (scoutState->photoSensorRight < scoutState->photoSensorLeft){
            /* Left is not an option */
            if (scoutState->photoSensorRight < scoutState->photoSensorFront){
                /* Front is not an option */
                if (scoutState->photoSensorRight < scoutState->photoSensorBack){
                    /* Right sensor is the lowest */
                    ScoutSerial::serialWrite("Right sensor lowest\n", 20);
                    performRotation(PHOTOPHOBIC_ROTATION);
                } else {
                    /* Back sensor is the lowest */
                    ScoutSerial::serialWrite("Back sensor lowest\n", 20);
                    scoutState->setSpeeds(-scoutState->forwardSpeed, -scoutState->forwardSpeed);
                    /* TODO: delay maybe necessary here to achieve sufficient movement */
                }
            } else {
                /* Right is not an option */
                if (scoutState->photoSensorFront < scoutState->photoSensorBack){
                    /* Front is the lowest */
                    ScoutSerial::serialWrite("Front sensor lowest\n", 20);
                    scoutState->setSpeeds(scoutState->forwardSpeed, scoutState->forwardSpeed);
                    /* TODO: delay maybe necessary here to achieve sufficient movement */

                } else {
                    /* Back is the lowest */
                    ScoutSerial::serialWrite("Back sensor lowest\n", 20);
                    scoutState->setSpeeds(-scoutState->forwardSpeed, -scoutState->forwardSpeed);
                    /* TODO: delay maybe necessary here to achieve sufficient movement */
                }
            }
        } else {
            /* Right is not an option */
            if (scoutState->photoSensorLeft < scoutState->photoSensorFront){
                /* Front is not an option */
                if (scoutState->photoSensorLeft < scoutState->photoSensorBack){
                    /* left sensor is the lowest */
                    ScoutSerial::serialWrite("Left sensor lowest\n", 20);
                    performRotation(-PHOTOPHOBIC_ROTATION);
                } else {
                    /* Back sensor is the lowest */
                    ScoutSerial::serialWrite("Back sensor lowest\n", 20);
                    scoutState->setSpeeds(-scoutState->forwardSpeed, -scoutState->forwardSpeed);
                    /* TODO: delay maybe necessary here to achieve sufficient movement */
                }
            } else {
                /* left is not an option */
                if (scoutState->photoSensorFront < scoutState->photoSensorBack){
                    /* Front is the lowest */
                    ScoutSerial::serialWrite("Front sensor lowest\n", 20);
                    scoutState->setSpeeds(scoutState->forwardSpeed, scoutState->forwardSpeed);
                    /* TODO: delay maybe necessary here to achieve sufficient movement */

                } else {
                    /* Back is the lowest */
                    ScoutSerial::serialWrite("Back sensor lowest\n", 20);
                    scoutState->setSpeeds(-scoutState->forwardSpeed, -scoutState->forwardSpeed);
                    /* TODO: delay maybe necessary here to achieve sufficient movement */
                }
            }
        }
    }
}


/* write Readings of all Photosensors to DEV port serial */
void debug_printPhotosensorReadings(){
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
bool readNewDestinations(int dest[]) {
    // read new destination entry
/* IMPORTANT:
 * Reading serial is what blows memory up, around 30% - should be disabled once we get close to 100% */

    int coordinates[2];

    bool newCoordinates = ScoutSerial::readCoordinates(coordinates);

    if (!newCoordinates) {
        return false;
    }

    dest[0] = coordinates[0];
    dest[1] = coordinates[1];

    return true;
}

/**
 * Perform rotation defined by degree
 */
void performRotation(int degrees) {
    float startAngle = scoutState->currentAngle;
    float factor = 0.0f;
    if (degrees < 0){
        factor = ((float) (-1 * degrees)) / 360.0f;
    } else {
        factor = ((float) degrees) / 360.0f;
    }
    bool loopCondition = true;

    while (loopCondition) {
        if (degrees == 0)
            return;

        if (degrees > 0) {
            // turn right
            scoutState->setSpeeds(scoutState->turningSpeed, -scoutState->turningSpeed);
            scoutState->updateRoboterPositionAndAngles();
        } else {
            // turn left
            scoutState->setSpeeds(-scoutState->turningSpeed, scoutState->turningSpeed);
            scoutState->updateRoboterPositionAndAngles();
        }

        /* rotation completed condition */
        if (scoutState->currentAngle > startAngle + 2 * M_PI * factor ||
            scoutState->currentAngle < startAngle - 2 * M_PI * factor) {
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


void operator delete(void *ptr) { free(ptr); }

void *operator new(size_t size) {
    return malloc(size);
}

void *operator new[](size_t size) {
    return malloc(size);
}