#include "DifferentialDrive.h"
#include <math.h>

DifferentialDrive::DifferentialDrive(float wheelRadius, float wheelDistance, float x, float y, float angle) {

    mWheelRadius = wheelRadius;
    mWheelDistance = wheelDistance;
    reset(x, y, angle);
}

void DifferentialDrive::reset(float x, float y, float angle) {
    mPosX = x;
    mPosY = y;
    mAngle = angle;
}

void DifferentialDrive::drive(float ul, float ur) {
    float z = (mWheelRadius * (ul + ur) / 2);
    float x_dot = z * cos(mAngle);
    float y_dot = z * sin(mAngle);
    float angle_dot = (mWheelRadius * (ul - ur) / mWheelDistance);

    mPosX += x_dot;
    mPosY += y_dot;
    mAngle += angle_dot;
}