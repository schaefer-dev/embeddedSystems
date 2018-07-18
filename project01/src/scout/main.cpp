
#include <OrangutanTime.h>
#include <OrangutanSerial.h>
#include "main.h"
#include <math.h>
#include <Pololu3pi.h>
#include "../utils/Coordinates.h"
#include "../scout/ScoutMonitor.h"
#include "ScoutRF.h"
#include "ScoutLineSensors.h"

bool spiEnabled = true;
int statusRF = 0;
int home[2] = {10, 10};
//CoordinateQueue *coordinateQueue;
ScoutState *scoutState;

void initialize() {
    // 1.1. Place the robot in the arena, 1 sec to do this
    delay(1000);

    // 1.2 Start calibrating
    // initialize serial connection
    ScoutSerial::initScoutSerial();
    OrangutanSerial::setBaudRate(9600);
    ScoutSerial::serialWrite("--- Start Serial Monitor ---\n", 29);

    /* initialization of Data structures */
    scoutState = new ScoutState();
    scoutState->resetDifferentialDrive(scoutState->currentX, scoutState->currentY, scoutState->currentAngle);
    scoutState->lastDiffDriveCall = millis();

#ifdef LINE_SENSOR_READINGS
    // Calibrate on dark arena
    ScoutLineSensors::init();
    ScoutLineSensors::calibrate(scoutState);
#endif

    // initialize differential updateRoboterPositionAndAngles
    scoutState->setSpeeds(0, 0);

#ifdef SCOUT_MONITOR
    ScoutMonitor::verifyState();
#endif

    delay(10);

    if (spiEnabled) {
        ScoutSPI::SPIMasterInit();
        delay(10);
        ScoutRF::initializeRFModule();
        delay(10);
    }

    ScoutSerial::serialWrite("SPI and RF Initialization complete\n", 36);
    // time to cancel and restart the robot
    delay(1000);

    // 2. Send a HELLO message to the referee on channel 111
    uint8_t payloadArray[2];
    payloadArray[0] = 0x42;
    payloadArray[1] = (uint8_t) (14);
    ScoutRF::sendMessageTo(ScoutRF::refereeAdress, payloadArray, 2);

    ScoutSerial::serialWrite("HELLO sent\n", 11);

    /*  Busy wait until config message received  */
    while (!scoutState->configurationReceived) {
        checkForNewRFMessage();
    }
    ScoutSerial::serialWrite("CONFIG received\n", 16);

    // wait until the light turns on
    delay(3100);

    int calibrationDuration = 2000;
    ScoutLineSensors::calibrate(scoutState, calibrationDuration);

    // wait until the light turns off
    ScoutSerial::serialWrite("Waiting for GO Message\n", 24);
    while (!scoutState->gameStarted) {
        checkForNewRFMessage();
    }

    ScoutSerial::serialWrite("GO!!!\n", 6);
}

int main() {
    initialize();

#ifdef DEBUG_SERIAL_PORT_ECHO
    char serialMessage[50];
    for (int i = 0; i < 50; i++) {
        serialMessage[i] = 32;
    }
    int serialMessageLength = 0;
#endif

#ifdef LINE_SENSOR_READINGS
    ScoutLineSensors::checkForLines(scoutState);
#endif

    while (1) {

        /* ALWAYS check for new RF Message */
        checkForNewRFMessage();


#ifdef SCENARIO_DEBUG_SEND_MESSAGES_CONTINIOUS
        int payload[10];
        payload[0] = 0x80;
        payload[1] = (int) 'T';
        payload[2] = (int) 'E';
        payload[3] = (int) 'S';
        payload[4] = (int) 'T';
        serialMessageLength = 5;
        ScoutRF::sendMessageTo(ScoutRF::collectorAdress, payload, serialMessageLength);
        ScoutSerial::serialWrite("sent\n", 5);
        delay(2000);
#endif


#ifdef DEBUG_SERIAL_PORT_ECHO
        serialMessageLength = ScoutSerial::readMessageFromSerial(serialMessage);
        if (serialMessageLength > 0) {
            ScoutSerial::serialWrite(serialMessage, serialMessageLength);
            ScoutSerial::serialWrite("\n", 1);
        }

        for (int i = 0; i < 50; i++){
            serialMessage[i] = 32;
        }
        delay(100);
#endif

        /*scoutState->checkForHighPhotoReadings();
        scoutState->navigate();
        delay(1);
        scoutState->updateRoboterPositionAndAngles();*/


#ifdef SCENARIO_RELAY
        delay(100);
        serialMessageLength = ScoutSerial::readMessageFromSerial(serialMessage);
        if (serialMessageLength != 0) {
            /* Message over serial was read */
            ScoutSerial::serialWrite("sending Message with Content: '", 31);
            ScoutSerial::serialWrite(serialMessage, serialMessageLength);
            ScoutSerial::serialWrite("' to collector\n", 15);

            int payload[serialMessageLength + 1];
            payload[0] = 0x80;
            for (int i = 0; i < serialMessageLength; i++) {
                payload[i + 1] = (int) serialMessage[i];
            }

            ScoutRF::sendMessageTo(ScoutRF::collectorAdress, payload, serialMessageLength + 1);

            /* clear message again */
            for (int i = 0; i < 31; i++) {
                serialMessage[i] = 32;
            }
        }
#endif


#ifdef SCENARIO_PHOTOPHOBIC
        /* photophobic mode */
        debug_printPhotosensorReadings();
        photophobicScout();
        delay(100);
#endif

    }

}


void receivePosUpdate(unsigned int angle, unsigned int x, unsigned int y) {
    float currentAngle = (float) angle / 10000.0f;
    currentAngle += 0.5f * M_PI;

    /* catch negative coordinates which are encoded to large ints */
    if (x > 2000)
        x = 0;
    if (y > 2000)
        y = 0;

    int currentX = ARENA_SIZE_X - x / 10.0f;
    int currentY = y / 10.0f;

    scoutState->resetDifferentialDrive(currentX, currentY, currentAngle);

    scoutState->destinationReached = true;
};


void checkForNewRFMessage() {
    statusRF = ScoutRF::queryRFModule();
    char messageReceived = statusRF & (1 << 6);

    if (messageReceived) {
        ScoutRF::processReceivedMessage(scoutState);
    }
}


bool checkForConfigMessage() {
    statusRF = ScoutRF::queryRFModule();
    char messageReceived = statusRF & (1 << 6);

    if (messageReceived) {
        ScoutRF::processReceivedMessage(scoutState);
    }

    if (ScoutRF::teamChannel != 0) {
        /*  Case for Config Message received  */
        ScoutRF::setTeamChannel(ScoutRF::teamChannel);
        ScoutSerial::serialWrite("Team channel changed accordingly.\n", 34);
        return true;
    }
    return false;
}


/* run photophobic Scout Controller as explained in Milestone 5
 * If all photosensors less then 100, dance in place, if all sensors in range of
 * defined threshold stand still, otherwise follow the shadow */
void photophobicScout() {
    scoutState->updatePhotoSensorReadings();

    if (scoutState->photoSensorLeft < photophobicDanceThreshold &&
        scoutState->photoSensorBack < photophobicDanceThreshold &&
        scoutState->photoSensorRight < photophobicDanceThreshold &&
        scoutState->photoSensorFront < photophobicDanceThreshold) {
        /* start dancing */
        while (true) {
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

        scoutState->setSpeeds(0, 0);
    } else {
        if (scoutState->photoSensorRight < scoutState->photoSensorLeft) {
            /* Left is not an option */
            if (scoutState->photoSensorRight < scoutState->photoSensorFront) {
                /* Front is not an option */
                if (scoutState->photoSensorRight < scoutState->photoSensorBack) {
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
                if (scoutState->photoSensorFront < scoutState->photoSensorBack) {
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
            if (scoutState->photoSensorLeft < scoutState->photoSensorFront) {
                /* Front is not an option */
                if (scoutState->photoSensorLeft < scoutState->photoSensorBack) {
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
                if (scoutState->photoSensorFront < scoutState->photoSensorBack) {
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
void debug_printPhotosensorReadings() {
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
 * Perform rotation defined by degree
 */
void performRotation(int degrees) {
    float startAngle = scoutState->currentAngle;
    float factor = 0.0f;
    if (degrees < 0) {
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


void operator delete(void *ptr) { free(ptr); }

void *operator new(size_t size) {
    return malloc(size);
}

void *operator new[](size_t size) {
    return malloc(size);
}
