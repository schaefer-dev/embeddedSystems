

#include <Zumo32U4LineSensors.h>
#include "CollectorState.h"

#ifndef EMBEDDEDSYSTEMS18_COLLECTORLINESENSORS_H
#define EMBEDDEDSYSTEMS18_COLLECTORLINESENSORS_H

#endif //EMBEDDEDSYSTEMS18_COLLECTORLINESENSORS_H

class CollectorLineSensors
{

public:
    CollectorLineSensors();

    CollectorLineSensors(CollectorState *collectorState);

    Zumo32U4LineSensors lineSensors;

    static void calibrate(CollectorState* collectorState);
    static bool detectLine();
    static void checkForLines();

};