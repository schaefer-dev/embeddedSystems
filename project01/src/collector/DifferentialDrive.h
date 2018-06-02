#ifndef EMBEDDEDSYSTEMS18_DIFFERENTIALDRIVE_H
#define EMBEDDEDSYSTEMS18_DIFFERENTIALDRIVE_H
#include "../../libs/libzumo/Zumo32U4Motors.h"


class DifferentialDrive : public Zumo32U4Motors {
public:
    static constexpr float WHEEL_RADIUS = 10;
    static constexpr float WHEEL_DISTANCE = 10;
    static float posX;
    static float posY ;
    static float angle;
    static int16_t leftSpeed;
    static int16_t rightSpeed;
    static void setLeftSpeed(int16_t speed);
    static void setRightSpeed(int16_t speed);
    static void flipLeftMotor(bool flip);
    static void flipRightMotor(bool flip);
    static void drive ();
    static void reset(float x,float y, float angle);

private:
    DifferentialDrive();
};


#endif //EMBEDDEDSYSTEMS18_DIFFERENTIALDRIVE_H
