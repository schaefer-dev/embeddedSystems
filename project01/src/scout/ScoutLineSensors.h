//
// Created by rafael on 06.07.18.
//

#ifndef EMBEDDEDSYSTEMS18_SCOUTLINESENSORS_H
#define EMBEDDEDSYSTEMS18_SCOUTLINESENSORS_H

#include "ScoutState.h"

class ScoutLineSensors
{

public:
    static void init();
    static void calibrate(ScoutState*);
    static bool detectLine();
    static void checkForLines(ScoutState*);
    static bool readNewLines();

private:
    ScoutLineSensors();
    static bool onLine;

};

#endif //EMBEDDEDSYSTEMS18_SCOUTLINESENSORS_H
