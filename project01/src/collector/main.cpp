#include <Arduino.h>
#include "../../libs/libzumo/Zumo32U4Motors.h"

void setup() {

}

void loop() {
    Zumo32U4Motors test = Zumo32U4Motors();
    test.setLeftSpeed(0);
}
