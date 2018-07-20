#ifndef Collector_state_H
#define Collector_state_H


#define OOB_PUNISH_TIME_MS 31000
#define DRIVE_BACKWARDS_TIME 600

#define DO_NOT_ROTATE_AGAIN_MS 500
#define NAV_NONE 0
#define NAV_AT_DESTINATION 1
#define NAV_TURNING_LEFT 2
#define NAV_TURNING_RIGHT 3
#define NAV_DRIVING_STRAIGHT 4


class CollectorState {

public:
    CollectorState();

    float currentX;
    float currentY;
    float currentAngle;
    float destinationX;
    float destinationY;
    bool isHarvestDestination;
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
    const int backwardsSpeed = -100;

    const int forwardSpeed = 100;
    const int turningSpeed = 100;

    // Calibration, forward: 100, turning: 100, straight: 0.55f, rotation: 0.35f
    // Calibration, forward: 150, turning: 150, straight: 0.6f, rotation: 0.435f

    // wenn zu weit dann wert erhöhen

    const float straightImprecision = 0.55f;     // simple approximation for friction when driving straight
    const float rotationImprecision = 0.35f;     // simple approximation for friction when turning

    bool configurationReceived = false;
    bool gameStarted = false;

    unsigned long harvestPositionReachedAtTime;
    unsigned long lastPositionUpdateAtTime;

    int scoutPosX, scoutPosY, scoutAngle;

    float getAngle();

    void resetDifferentialDrive(float x, float y, float a);

    void setSpeeds(int newLeftSpeed, int newRightSpeed);

    void updateRoboterPositionAndAngles();

    void outOfBoundsMessage();

    void harvestPositionMessage(int, int, int);

    void navigate();

    void generateDestination();

    void sendPosToTeammate();

    void danceBlocking();

    void scoutPositionMessage(float angle, float x, float y);

private:
    static constexpr float WHEEL_RADIUS = 1.75f;
    static constexpr float WHEEL_DISTANCE = 9.3f;

};

#endif
