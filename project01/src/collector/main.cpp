#include <Arduino.h>
#include "DifferentialDrive.h"
#include "Collector_state.h"
#include "Coordinates.h"

int ByteReceived;

Coordinate_queue *c_queue;

Collector_state *c_state;

void setup() {
    // initialize differential drive
    DifferentialDrive::reset(0,0,0);
    DifferentialDrive::setLeftSpeed(leftSpeed);
    DifferentialDrive::setRightSpeed(rightSpeed);

    c_state = new Collector_state();

    c_queue = new Coordinate_queue();
    c_queue->append(50,50);

    // initialize destination
    c_state->destination_x = 50;
    c_state->destinationY = 50;

    // initialize serial connection
    Serial1.begin(9600);
    Serial1.println("--- Start Serial Monitor ---");
    Serial1.println("(Decimal)(Hex)(Character)");
    Serial1.println();
}

void loop() {
    // drive to destination
    thetaCorrection();
    DifferentialDrive::drive();

    // read communication
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


