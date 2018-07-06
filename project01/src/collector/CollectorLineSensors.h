

#include <Zumo32U4LineSensors.h>
#include "CollectorState.h"

#ifndef EMBEDDEDSYSTEMS18_COLLECTORLINESENSORS_H
#define EMBEDDEDSYSTEMS18_COLLECTORLINESENSORS_H

class CollectorLineSensors
{

public:
    static void init(CollectorState* collectorState);
    static void calibrate(CollectorState* collectorState);
    static bool detectLine();
    static void checkForLines();
    static Zumo32U4LineSensors lineSensors;

private:
    CollectorLineSensors();
};

#endif //EMBEDDEDSYSTEMS18_COLLECTORLINESENSORS_H
