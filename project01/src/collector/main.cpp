#include <Arduino.h>
#include "CollectorState.h"
#include "Coordinates.h"
#include "main.h"

int ByteReceived;

CoordinateQueue *coordinateQueue;
CollectorState *collectorState;
int rotationCounter;

void setup() {

    /* initialization of Data structures */
    collectorState = new CollectorState();
    coordinateQueue = new CoordinateQueue();
    rotationCounter = 0;

    // initialize differential updateRoboterPositionAndAngles
    collectorState->setSpeeds(0,0);
    collectorState->resetDifferentialDrive(0, 0, 0);

    // initialize serial connection
    Serial1.flush();
    Serial1.begin(9600);
    Serial1.flush();
    Serial1.println("--- Start Serial Monitor ---");
    Serial1.println();


    // initialize destination
    collectorState->destinationX = 30;
    collectorState->destinationY = 30;
    coordinateQueue->append(50,50);

    collectorState->lastDiffDriveCall = millis();
}

void loop() {
    if(rotationCounter == 0)
        performStraightDrive(60);
    rotationCounter = 1;

}


/* ------------------------------------------------------------*/
/* ----------------- HELPER FUNCTIONS -------------------------*/
/* ------------------------------------------------------------*/

void driveToDestination(){
    performRotation();
}

void readNewDestinations(){
    // read new destination entry
    if (Serial1.available() > 0)
    {
        ByteReceived = Serial1.read();
        Serial1.print(ByteReceived);
        Serial1.print("        ");
        Serial1.print(ByteReceived, HEX);
        Serial1.print("       ");
        Serial1.print(char(ByteReceived));
        Serial1.println();
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


