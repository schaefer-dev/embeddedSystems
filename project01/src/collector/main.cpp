#include <Arduino.h>
#include "CollectorState.h"
#include "Coordinates.h"

int ByteReceived;

CoordinateQueue *coordinateQueue;
CollectorState *collectorState;

void setup() {

    collectorState = new CollectorState();
    coordinateQueue = new CoordinateQueue();
    coordinateQueue->append(50,50);

    // initialize differential drive
    collectorState->resetDifferentialDrive(0, 0, 0);
    collectorState->setLeftSpeed(collectorState->leftSpeed);
    collectorState->setRightSpeed(collectorState->rightSpeed);

    // initialize destination
    collectorState->destinationX = 30;
    collectorState->destinationY = 30;

    // initialize serial connection
    Serial1.flush();
    Serial1.begin(9600);
    Serial1.flush();
    Serial1.println("--- Start Serial Monitor ---");
    Serial1.println();
}

void loop() {

    // drive to destination
    collectorState->thetaCorrection();

    delay(100);

    // Some problem with diff drive?
    collectorState->drive();

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


