#ifndef Collector_state_H
#define Collector_state_H


#define OOB_PUNISH_TIME_MS 10000


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
    bool destinationReached;
    bool outOfBounds;
    unsigned long outOfBoundsTime;

    const int forwardSpeed = 200;
    const int turningSpeed = 100;

    /* for base speed rotation of 100, 0.47f perfect */
    /* for base speed rotation of 200, 0.55f perfect */
    const float rotationImprecision = 0.47f;     // simple approximation for friction when turning
    const float straightImprecision = 0.665f;     // simple approximation for friction when driving straight

    float getAngle();
    bool navigateToDestination();
    void resetDifferentialDrive(float x, float y, float a);
    void setSpeeds(int newLeftSpeed, int newRightSpeed);
    void updateRoboterPositionAndAngles();
    void outOfBoundsMessage();

private:
    static constexpr float WHEEL_RADIUS = 1.75f;
    static constexpr float WHEEL_DISTANCE = 9.3f;

};

#endif