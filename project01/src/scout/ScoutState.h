//
// Created by Daniel Schäfer on 06.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_SCOUTSTATE_H
#define EMBEDDEDSYSTEMS18_SCOUTSTATE_H

#include "../utils/Coordinates.h"

#define OOB_PUNISH_TIME_MS 10000

#define DO_NOT_ROTATE_AGAIN_MS 500
#define NAV_NONE 0
#define NAV_AT_DESTINATION 1
#define NAV_TURNING_LEFT 2
#define NAV_TURNING_RIGHT 3
#define NAV_DRIVING_STRAIGHT 4

class ScoutState
{

public:
    ScoutState();

    float currentX;
    float currentY;
    float currentAngle;
    float destinationX;
    float destinationY;
    float nextDestinationX;
    float nextDestinationY;
    unsigned short nextDestinationCounter;
    int leftSpeed;
    int rightSpeed;
    long lastDiffDriveCall;
    bool destinationReached;
    bool outOfBounds;
    unsigned long outOfBoundsTime;
    bool drivingDisabled;
    unsigned long earliestNextRotationTime;
    short navigationStep;

    /* last photosensorReadings */
    int photoSensorLeft;
    int photoSensorFront;
    int photoSensorRight;
    int photoSensorBack;
    long lastPhotoSensorUpdate;

    const int forwardSpeed = 160;
    const int turningSpeed = 100;

    const float rotationImprecision = 0.86f;     // simple approximation for friction when turning
    const float straightImprecision = 0.8f;     // simple approximation for friction when driving straight

    float getAngle();
    void resetDifferentialDrive(float x, float y, float a);
    void setSpeeds(int newLeftSpeed, int newRightSpeed);
    void updateRoboterPositionAndAngles();
    void updatePhotoSensorReadings();
    void outOfBoundsMessage();
    void navigate();

private:
    /* TODO */
    static constexpr float WHEEL_RADIUS = 1.75f;
    static constexpr float WHEEL_DISTANCE = 9.3f;

};

#endif //EMBEDDEDSYSTEMS18_SCOUTSTATE_H