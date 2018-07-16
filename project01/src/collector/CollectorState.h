#ifndef Collector_state_H
#define Collector_state_H


#define OOB_PUNISH_TIME_MS 10000
#define DRIVE_BACKWARDS_TIME 500

#define DO_NOT_ROTATE_AGAIN_MS 500
#define NAV_NONE 0
#define NAV_AT_DESTINATION 1
#define NAV_TURNING_LEFT 2
#define NAV_TURNING_RIGHT 3
#define NAV_DRIVING_STRAIGHT 4


class CollectorState
{

public:
    CollectorState();

    float currentX;
    float currentY;
    float currentAngle;
    float destinationX;
    float destinationY;
    float nextDestinationX;
    float nextDestinationY;
    bool isHarvestDestination;
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

    bool unhandledCollisionFlag;
    unsigned long driveBackwardsUntil;
    const int backwardsSpeed = -30;

    const int forwardSpeed = 100;
    const int turningSpeed = 100;

    /* for base speed rotation of 100, 0.47f perfect */
    /* for base speed rotation of 200, 0.55f perfect */
    const float rotationImprecision = 0.47f;     // simple approximation for friction when turning
    const float straightImprecision = 0.665f;     // simple approximation for friction when driving straight

    bool configurationReceived = false;
    bool gameStarted = false;

    int scoutPosX, scoutPosY, scoutAngle;

    float getAngle();
    void resetDifferentialDrive(float x, float y, float a);
    void setSpeeds(int newLeftSpeed, int newRightSpeed);
    void updateRoboterPositionAndAngles();
    void outOfBoundsMessage();
    void harvestPositionMessage(int, int, int);
    void navigate();
    void generateDestination();
    void sendPositionUpdate();
    void danceBlocking();

    void scoutPositionMessage(float angle, float x, float y);

private:
    static constexpr float WHEEL_RADIUS = 1.75f;
    static constexpr float WHEEL_DISTANCE = 9.3f;

};

#endif
