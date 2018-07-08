//
// Created by U500304 on 18.06.2018.
//

#ifndef EMBEDDEDSYSTEMS18_MONITOR_H
#define EMBEDDEDSYSTEMS18_MONITOR_H

#include "main.h"

#ifdef SCOUT_MONITOR

class ScoutMonitor {
private:
    static unsigned long lastBufferEmpty;
    static char scoutHarvestingState;
    static char scoutReactToPingState;
    static char scoutEmptyBufferState;

public:
    static void logSendHarvest();

    static void logPingScout();

    static void logPongScout();

    static void emptyBuffer();

    static void verifyState();

    static void getStatus(char*);
};

#endif
#endif //EMBEDDEDSYSTEMS18_MONITOR_H
