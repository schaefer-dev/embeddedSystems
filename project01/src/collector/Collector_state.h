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

    const int baseSpeed = 100;


    float getAngle();

    void thetaCorrection();
    void diff_drive_reset(float x, float y, float a);
    void setLeftSpeed(int speed);
    void setRightSpeed(int speed);
    void drive();

private:
    static constexpr float WHEEL_RADIUS = 1.75f;
    static constexpr float WHEEL_DISTANCE = 9.3f;

};

#endif