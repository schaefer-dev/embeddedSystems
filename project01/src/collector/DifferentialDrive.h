#ifndef EMBEDDEDSYSTEMS18_DIFFERENTIALDRIVE_H
#define EMBEDDEDSYSTEMS18_DIFFERENTIALDRIVE_H
#include "../../libs/libzumo/Zumo32U4Motors.h"
#include "Collector_state.h"


class DifferentialDrive : public Zumo32U4Motors {
public:
    static void drive ();
    static Collector_state *c_state;
    static void initialize_diff_drive(Collector_state *);
    static void setLeftSpeed(int speed);
    static void setRightSpeed(int speed);
    static void reset(float x,float y, float angle);


private:
    DifferentialDrive();
    static constexpr float WHEEL_RADIUS = 10;
    static constexpr float WHEEL_DISTANCE = 10;
    static void flipLeftMotor(bool flip);
    static void flipRightMotor(bool flip);
};


#endif //EMBEDDEDSYSTEMS18_DIFFERENTIALDRIVE_H
