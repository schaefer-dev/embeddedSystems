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

    float left_speed;
    float right_speed;

    const int16_t baseSpeed = 50;

    float getAngle();

    void thetaCorrection();

private:

};

#endif