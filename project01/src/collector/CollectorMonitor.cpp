//
// Created by U500304 on 18.06.2018.
//

#include "CollectorMonitor.h"

char CollectorMonitor::collectorCheckProximityState;
char CollectorMonitor::collectorReactToPingState;

bool CollectorMonitor::atHarvest;
bool CollectorMonitor::checkProximity;


void CollectorMonitor::logAtHarvest(bool state) {
    atHarvest = state;

    if (collectorCheckProximityState == 0) {
        if (!(atHarvest || checkProximity)) {
            collectorCheckProximityState = 1;
            // bad state
        }
    }
}

void CollectorMonitor::logCheckProximity(bool state) {
    checkProximity = state;

    if (collectorCheckProximityState == 0) {
        if (!(atHarvest || checkProximity)) {
            collectorCheckProximityState = 1;
            // bad state
        }
    }
}

char CollectorMonitor::verifyState() {
    bool bad = true;
    bad &= collectorCheckProximityState == 1;
    bad &= collectorReactToPingState == 4;
    bad &= collectorReactToPingState == 4;
    if (bad) return 0;

    bool prelimGood = true;
    prelimGood &= collectorCheckProximityState == 0;
    prelimGood &= collectorReactToPingState < 4;
    if (prelimGood) return 1;

    return 3;   // should not happen
}


void CollectorMonitor::logPingCollector() {
    if (collectorReactToPingState < 4) {
        collectorReactToPingState++;
    }
}

void CollectorMonitor::logPongCollector() {
    if (collectorReactToPingState < 4) {
        collectorReactToPingState = 0;
    }
}