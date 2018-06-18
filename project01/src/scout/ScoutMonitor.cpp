//
// Created by U500304 on 18.06.2018.
//

#include "ScoutMonitor.h"

char ScoutMonitor::scoutHarvestingState;
char ScoutMonitor::scoutReactToPingState;

void ScoutMonitor::logSendHarvest() {
    scoutHarvestingState = 1;
}

char ScoutMonitor::verifyState() {
    bool prelimGood = true;
    prelimGood &= scoutReactToPingState < 4;
    if (prelimGood) return 1;

    bool good = true;
    good &= scoutHarvestingState == 1;
    if (good) return 2;

    return 3;   // should not happen
}

void ScoutMonitor::logPingScout() {
    if (scoutReactToPingState < 4) {
        scoutReactToPingState++;
    }
}

void ScoutMonitor::logPongScout() {
    if (scoutReactToPingState < 4) {
        scoutReactToPingState = 0;
    }
}
