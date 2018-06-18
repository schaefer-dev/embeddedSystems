//
// Created by U500304 on 18.06.2018.
//

#ifndef EMBEDDEDSYSTEMS18_COLLECTORMONITOR_H
#define EMBEDDEDSYSTEMS18_COLLECTORMONITOR_H


class CollectorMonitor {
private:
    static char collectorCheckProximityState;
    static char collectorReactToPingState;

    static bool atHarvest;
    static bool checkProximity;

public:
    static void logAtHarvest(bool);
    static void logCheckProximity(bool);
    static void logPingCollector();
    static void logPongCollector();

    static char verifyState();
};


#endif //EMBEDDEDSYSTEMS18_COLLECTORMONITOR_H
