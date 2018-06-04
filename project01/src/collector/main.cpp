#include <Arduino.h>
#include "Collector_state.h"
#include "Coordinates.h"

int ByteReceived;

Coordinate_queue *c_queue;

Collector_state *c_state;

void setup() {

    c_state = new Collector_state();

    c_queue = new Coordinate_queue();
    c_queue->append(50,50);

    // initialize differential drive
    c_state->diff_drive_reset(0,0,0);
    c_state->setLeftSpeed(c_state->left_speed);
    c_state->setRightSpeed(c_state->right_speed);

    // initialize destination
    c_state->destination_x = 50;
    c_state->destination_y = 50;

    // initialize serial connection
    Serial1.flush();
    Serial1.begin(9600);
    Serial1.flush();
    Serial1.println("--- Start Serial Monitor ---");
    Serial1.println("(Decimal)(Hex)(Character)");
    Serial1.println();
}

void loop() {

    // drive to destination
    c_state->thetaCorrection();

    // Some problem with diff drive?
    c_state->drive();
    delay(100);

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


