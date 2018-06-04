#include <stddef.h>

#ifndef Collector_state_H
#define Collector_state_H

class CollectorState
{

public:
    CollectorState();

    float currentX;
    float currentY;
    float currentAngle;
    float destinationX;
    float destinationY;
    int leftSpeed;
    int rightSpeed;

    const int baseSpeed = 100;

    float getAngle();
    void thetaCorrection();
    void resetDifferentialDrive(float x, float y, float a);
    void setLeftSpeed(int speed);
    void setRightSpeed(int speed);
    void drive();

private:
    static constexpr float WHEEL_RADIUS = 1.75f;
    static constexpr float WHEEL_DISTANCE = 9.3f;

};

#endif