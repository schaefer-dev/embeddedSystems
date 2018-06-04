#include "DifferentialDrive.h"
#include <math.h>

Collector_state *collector_state;

void DifferentialDrive::initialize_diff_drive(Collector_state *collector_state){
    collector_state = collector_state;
}

void DifferentialDrive::reset(float x, float y, float a) {
    collector_state->current_x = x;
    collector_state->current_y = y;
    collector_state->current_angle = a;
}

void DifferentialDrive::setLeftSpeed(int speed) {
    Zumo32U4Motors::setLeftSpeed(speed);
    collector_state->left_speed = speed;
}

void DifferentialDrive::setRightSpeed(int speed) {
    Zumo32U4Motors::setRightSpeed(speed);
    collector_state->right_speed = speed;
}

void DifferentialDrive::flipLeftMotor(bool flip) {
    Zumo32U4Motors::flipLeftMotor(flip);
    collector_state->left_speed *= -1;
}

void DifferentialDrive::flipRightMotor(bool flip) {
    Zumo32U4Motors::flipRightMotor(flip);
    collector_state->right_speed *= -1;
}

void DifferentialDrive::drive() {
    float z = (WHEEL_RADIUS * (collector_state->left_speed + collector_state->left_speed) / 2);
    float x_dot = z * cos(collector_state->current_angle);
    float y_dot = z * sin(collector_state->current_angle);
    float angle_dot = (WHEEL_RADIUS * (collector_state->left_speed - collector_state->right_speed) / WHEEL_DISTANCE);

    collector_state->current_x += x_dot;
    collector_state->current_y += y_dot;
    collector_state->current_angle += angle_dot;

    collector_state->current_x *= 0.1f;
    collector_state->current_y *= 0.1f;
    collector_state->current_angle *= 0.1f;

    /* TODO
     * this is not working, needs to know time since last call
     * such that concrete differntial equation is correct */
}

DifferentialDrive::DifferentialDrive() = default;

