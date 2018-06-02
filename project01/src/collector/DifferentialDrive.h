#ifndef EMBEDDEDSYSTEMS18_DIFFERENTIALDRIVE_H
#define EMBEDDEDSYSTEMS18_DIFFERENTIALDRIVE_H
#include "../../libs/libzumo/Zumo32U4Motors.h"


class DifferentialDrive : public Zumo32U4Motors {
public:
    float mPosX;
    float mPosY;
    float mAngle;
    float mWheelRadius;
    float mWheelDistance;
    DifferentialDrive(float wheelRadius, float wheelDistance, float x,float y, float angle);
    void drive (float ul, float ur);
    void reset(float x,float y, float angle);
};


#endif //EMBEDDEDSYSTEMS18_DIFFERENTIALDRIVE_H
