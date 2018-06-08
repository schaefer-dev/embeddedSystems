//
// Created by Daniel Schäfer on 06.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_SCOUTSTATE_H
#define EMBEDDEDSYSTEMS18_SCOUTSTATE_H

class ScoutState
{

public:
    ScoutState();

    float currentX;
    float currentY;
    float currentAngle;
    float destinationX;
    float destinationY;
    int leftSpeed;
    int rightSpeed;
    long lastDiffDriveCall;
    bool destinationReached;

    const int forwardSpeed = 120;
    const int turningSpeed = 70;

    /* TODO */
    const float rotationImprecision = 0.47f;     // simple approximation for friction when turning
    const float straightImprecision = 0.665f;     // simple approximation for friction when driving straight

    float getAngle();
    bool navigateToDestination();
    void resetDifferentialDrive(float x, float y, float a);
    void setSpeeds(int newLeftSpeed, int newRightSpeed);
    void updateRoboterPositionAndAngles();

private:
    /* TODO */
    static constexpr float WHEEL_RADIUS = 1.75f;
    static constexpr float WHEEL_DISTANCE = 9.3f;

};

#endif //EMBEDDEDSYSTEMS18_SCOUTSTATE_H