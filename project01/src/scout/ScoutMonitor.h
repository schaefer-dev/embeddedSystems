//
// Created by U500304 on 18.06.2018.
//

#ifndef EMBEDDEDSYSTEMS18_MONITOR_H
#define EMBEDDEDSYSTEMS18_MONITOR_H

class ScoutMonitor {
private:
    static char scoutHarvestingState;
    static char scoutReactToPingState;

public:
    static void logSendHarvest();
    static void logPingScout();
    static void logPongScout();

    static char verifyState();
};


#endif //EMBEDDEDSYSTEMS18_MONITOR_H
