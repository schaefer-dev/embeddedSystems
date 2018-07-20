//
// Created by Daniel Schäfer on 06.06.18.
//

#ifndef EMBEDDEDSYSTEMS18_SCOUTSTATE_H
#define EMBEDDEDSYSTEMS18_SCOUTSTATE_H

#include "../utils/Coordinates.h"

#define OOB_PUNISH_TIME_MS 31000
#define DRIVE_BACKWARDS_TIME 500

#define PHOTOSENSOR_TRESHOLD 100

#define DO_NOT_ROTATE_AGAIN_MS 200
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
    int calibratedLightSensorThreshhold;


    unsigned long photoSensorTimer;
    int photoSensorCurrentMax;
    float photoX;
    float photoY;


    float collectorX;
    float collectorY;
    float collectorAngle;

    bool unhandledCollisionFlag;
    unsigned long driveBackwardsUntil;
    const int backwardsSpeed = -20;


    const int forwardSpeed = 50;
    const int turningSpeed = 30;

    // Calibrated for forward=80, turning=50  rotation=2.2 and straight=2.3
    // Calibrated for forward=50, turning=30, rotation=2.1 and straight=2.3
    const float rotationImprecision = 2.1f;     // simple approximation for friction when turning
    const float straightImprecision = 2.3f;     // simple approximation for friction when driving straight

    bool configurationReceived = false;
    bool gameStarted = false;


    float getAngle();
    void resetDifferentialDrive(float x, float y, float a);
    void setSpeeds(int newLeftSpeed, int newRightSpeed);
    void updateRoboterPositionAndAngles();
    void updatePhotoSensorReadings();
    void handleHighPhotoReadings();
    void outOfBoundsMessage();
    void navigate();
    void generateDestination();

private:
    static constexpr float WHEEL_RADIUS = 1.75f;
    static constexpr float WHEEL_DISTANCE = 9.3f;


    void sendPosToTeammate();
};

#endif //EMBEDDEDSYSTEMS18_SCOUTSTATE_H
