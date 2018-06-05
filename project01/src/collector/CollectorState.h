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
    long lastDiffDriveCall;

    const int baseSpeed = 100;
    const float timefactor = 0.10f;             // linear scale for differential equation
    const float rotationImprecision = 0.7f;     // simple approximation for friction when turning

    float getAngle();
    void navigateToDestination();
    void resetDifferentialDrive(float x, float y, float a);
    void setLeftSpeed(int speed);
    void setRightSpeed(int speed);
    void updateRoboterPositionAndAngles();

private:
    static constexpr float WHEEL_RADIUS = 1.75f;
    static constexpr float WHEEL_DISTANCE = 9.3f;

};

#endif