#include <stddef.h>

#ifndef Collector_state_H
#define Collector_state_H

class Collector_state
{

public:
    //constructor
    Collector_state();

    //deconstructor?

    float current_x;
    float current_y;
    float current_angle;

    float destination_x;
    float destination_y;

    int left_speed;
    int right_speed;

    const int baseSpeed = 50.0f;

    float getAngle();

    void thetaCorrection();

private:

};

#endif