//
// Created by rafael on 06.07.18.
//

#ifndef EMBEDDEDSYSTEMS18_SCOUTLINESENSORS_H
#define EMBEDDEDSYSTEMS18_SCOUTLINESENSORS_H

#include <stdint-gcc.h>
#include "ScoutState.h"

class ScoutLineSensors
{

public:
    static void init();
    static void calibrate(ScoutState*, int duration = 2500);
    static bool detectLine();
    static void checkForLines(ScoutState*);
    static bool readNewLines();

private:
    ScoutLineSensors();
    static bool onLine;
    static const uint8_t threshold = 400;

};

#endif //EMBEDDEDSYSTEMS18_SCOUTLINESENSORS_H
