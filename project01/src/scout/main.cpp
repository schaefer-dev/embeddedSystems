
#include <OrangutanTime.h>
#include <OrangutanSerial.h>
#include "main.h"
#include <math.h>
#include "../utils/Coordinates.h"
#include "../scout/ScoutMonitor.h"
#include "ScoutRF.h"
#include <Pololu3pi.h>


bool spiEnabled = true;
int statusRF = 0;
int home[2] = {160, 50};      // home

CoordinateQueue *coordinateQueue;
ScoutState *scoutState;


void initialize(){
    /* initialization of Data structures */
    scoutState = new ScoutState();
    coordinateQueue = new CoordinateQueue();

    // initialize differential updateRoboterPositionAndAngles
    scoutState->setSpeeds(0, 0);
    scoutState->resetDifferentialDrive(0, 0, 0);


    /* init line sensors */
    pololu_3pi_init(2000);

    for (int i = 0; i < 80; i++) {
        if(i < 20 || i >= 60)
            set_motors(40,-40);
        else
            set_motors(-40,40);
        calibrate_line_sensors(IR_EMITTERS_ON);
        delay_ms(20);
    }
    set_motors(0,0);


#ifdef DEBUG
    // initialize serial connection
    ScoutSerial::initScoutSerial();
    OrangutanSerial::setBaudRate(9600);
    ScoutSerial::serialWrite("--- Start Serial Monitor ---\n", 29);
#endif

    /* SETUP */
#ifdef SCOUT_MONITOR
    ScoutMonitor::verifyState();
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

    ScoutSerial::serialWrite("\nInitialization complete\n",25);

}

int main() {

    initialize();

    char serialMessage[50];
    for (int i = 0; i < 50; i++){
        serialMessage[i] = 32;
    }
    int serialMessageLength = 0;

    while (1) {

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


#ifdef LINE_SENSOR_READINGS
        checkForLines();
#endif

#ifdef SCENARIO_HOMING
        homing();
#endif

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
        checkForNewRFMessage();

#ifdef SCENARIO_PHOTOPHOBIC
        /* photophobic mode */
        debug_printPhotosensorReadings();
        photophobicScout();
        delay(100);
#endif

    }

}

void checkForLines() {

    unsigned int sensorReadings[5];

    unsigned int position = read_line(sensorReadings, IR_EMITTERS_ON);

    ScoutSerial::serialWriteInt(position);

    for (int i = 0; i < 5; i++){
        ScoutSerial::serialWriteInt(sensorReadings[i]);
    }

    delay(1000);

}

void homing() {

    if (!driveToDestination()){
        // already home -> dancing
        /*
        performRotation(90);
        performRotation(-180);
        performRotation(90);
         */
    }
    delay(1);
    scoutState->updateRoboterPositionAndAngles();
}

void receivePosUpdate(unsigned int angle, unsigned int x, unsigned int y){
    float currentAngle = (float)angle / 10000.0f;
    currentAngle += 0.5f * M_PI;
    int currentX = ARENA_SIZE_X - x / 10.0f;
    int currentY = y / 10.0f;

    scoutState->resetDifferentialDrive(currentX, currentY, currentAngle);


    if (coordinateQueue->isEmpty()){
        coordinateQueue->append(home[0], home[1]);
    }
};


void checkForNewRFMessage(){
    statusRF = ScoutRF::queryRFModule();
    char messageReceived = statusRF & (1 << 6);

    if (messageReceived){
        ScoutRF::processReceivedMessage(scoutState);
    }
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

void debug_sendPingToCollector(){

}


void operator delete(void *ptr) { free(ptr); }

void *operator new(size_t size) {
    return malloc(size);
}

void *operator new[](size_t size) {
    return malloc(size);
}