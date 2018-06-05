#include <Arduino.h>
#include "CollectorState.h"
#include "Coordinates.h"
#include "main.h"

int ByteReceived;

CoordinateQueue *coordinateQueue;
CollectorState *collectorState;
int rotationCounter;
bool endProgram;

void setup() {

    /* initialization of Data structures */
    collectorState = new CollectorState();
    coordinateQueue = new CoordinateQueue();
    rotationCounter = 0;

    // initialize differential updateRoboterPositionAndAngles
    collectorState->setSpeeds(0,0);
    collectorState->resetDifferentialDrive(0, 0, 0);
    endProgram = false;

    // initialize serial connection
    Serial1.flush();
    Serial1.begin(9600);
    Serial1.flush();
    Serial1.println("--- Start Serial Monitor ---");
    Serial1.println();


    collectorState->lastDiffDriveCall = millis();
}

void loop() {

    /* Default roboter code */
    readNewDestinations();
    driveToDestination();
    collectorState->updateRoboterPositionAndAngles();


    /* Testing Code: */
}


/* ------------------------------------------------------------*/
/* ----------------- HELPER FUNCTIONS -------------------------*/
/* ------------------------------------------------------------*/

bool driveToDestination(){
    if (collectorState->destinationReached) {
        struct CoordinateQueue::CoordinateNode *node = coordinateQueue->pop();
        if (node == nullptr){
            collectorState->setSpeeds(0,0);
            return false;
        }

        collectorState->destinationX = node->x;
        collectorState->destinationY = node->y;
        collectorState->destinationReached = false;
    }
    return collectorState->navigateToDestination();
}

void readNewDestinations(){
    // read new destination entry
    if (Serial1.available() > 2)
    {
        int xDestination = 0;
        int yDestination = 0;

        xDestination = Serial1.parseInt();
        yDestination = Serial1.parseInt();

        Serial1.print("NEW DESTINATION IN QUEUE: (");
        Serial1.print(xDestination);
        Serial1.print(", ");
        Serial1.print(yDestination);
        Serial1.println(")");
        Serial1.flush();

        coordinateQueue->append(xDestination,yDestination);

    }
}

/* perform 1 rotation */
void performRotation(){
    float startAngle = collectorState->currentAngle;
    bool loopCondition = true;

    while(loopCondition){

        // turn right
        collectorState->setSpeeds(collectorState->baseSpeed, -collectorState->baseSpeed);

        delay(10);

        collectorState->updateRoboterPositionAndAngles();

        /* rotation completed condition */
        if (collectorState->currentAngle > startAngle + 2 * M_PI || collectorState->currentAngle < startAngle - 2 * M_PI) {
            loopCondition = false;
            Serial1.println("one rotation performed!");
            collectorState->setSpeeds(0, 0);

        } else {
            Serial1.println(collectorState->currentAngle);
        }
    }
}

/* perform 1 rotation */
void performStraightDrive(int cmLength){
    float startX = collectorState->currentX;
    float targetX = startX + cmLength;
    bool loopCondition = true;

    while(loopCondition){

        // drive straight
        collectorState->setSpeeds(collectorState->baseSpeed, collectorState->baseSpeed);

        delay(10);

        collectorState->updateRoboterPositionAndAngles();

        /* rotation completed condition */
        if (collectorState->currentX > targetX) {
            loopCondition = false;
            Serial1.println("Driving performed!");
            collectorState->setSpeeds(0, 0);

        } else {
            Serial1.println(collectorState->currentX);
        }
    }
}


