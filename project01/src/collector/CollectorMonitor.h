#ifndef EMBEDDEDSYSTEMS18_COLLECTORMONITOR_H
#define EMBEDDEDSYSTEMS18_COLLECTORMONITOR_H

#include "Arduino.h"

class CollectorMonitor {
private:
    static unsigned long lastBufferEmpty;
    static char collectorCheckProximityState;
    static char collectorReactToPingState;
    static char collectorEmptyBufferState;

    static bool atHarvest;
    static bool checkProximity;

public:
    static void logAtHarvest(bool);

    static void logCheckProximity(bool);

    static void logPingCollector();

    static void logPongCollector();

    static void emptyBuffer();

    static void verifyState();

    static void getStatus(char*);
};

#endif //EMBEDDEDSYSTEMS18_COLLECTORMONITOR_H
