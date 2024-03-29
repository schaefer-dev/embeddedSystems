

#include <Zumo32U4LineSensors.h>
#include "CollectorState.h"

#ifndef EMBEDDEDSYSTEMS18_COLLECTORLINESENSORS_H
#define EMBEDDEDSYSTEMS18_COLLECTORLINESENSORS_H

class CollectorLineSensors
{

public:
    static void init(CollectorState* collectorState);
    static void calibrate(CollectorState* collectorState, int calibrationDuration = 2500);
    static bool detectLine();
    static void driveOverLines(CollectorState* collectorState);

private:
    static Zumo32U4LineSensors lineSensors;
    static bool onLine;
    static const uint8_t threshold = 500;

    CollectorLineSensors();
};

#endif //EMBEDDEDSYSTEMS18_COLLECTORLINESENSORS_H
