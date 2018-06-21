#include "ScoutMonitor.h"

#ifdef SCOUT_MONITOR

#include "ScoutSerial.h"
#include "OrangutanTime.h"

char ScoutMonitor::scoutHarvestingState;
char ScoutMonitor::scoutReactToPingState;
char ScoutMonitor::scoutEmptyBufferState;
unsigned long ScoutMonitor::lastBufferEmpty;

void ScoutMonitor::logSendHarvest() {
    if (scoutHarvestingState < 1) {
        scoutHarvestingState = 1;
        ScoutSerial::serialWrite("Good trace. Sent at least one harvesting position.", 50);
    }
}

void ScoutMonitor::logPingScout() {
    if (scoutReactToPingState < 4) {
        scoutReactToPingState++;
    }
    if (scoutReactToPingState == 4) {
        ScoutSerial::serialWrite("Bad trace. Ignored 3 consecutive PING messages.", 47);
    }
}

void ScoutMonitor::logPongScout() {
    if (scoutReactToPingState < 4) {
        scoutReactToPingState = 0;
    }
}

void ScoutMonitor::emptyBuffer() {
    unsigned long now = millis();
    if (now - lastBufferEmpty > 500) {
        scoutEmptyBufferState = 1;
        ScoutSerial::serialWrite("Bad trace. Did not empty buffer for more than 500ms.", 52);
    }
    lastBufferEmpty = now;
}

void ScoutMonitor::verifyState() {
    if (scoutReactToPingState == 4)
        ScoutSerial::serialWrite("Bad trace. Ignored 3 consecutive PING messages.", 47);
    else
        ScoutSerial::serialWrite("Prelim good trace. Never Ignored 3 consecutive PING messages.", 61);

    if (scoutHarvestingState == 1)
        ScoutSerial::serialWrite("Good trace. Sent at least one harvesting position.", 50);
    else
        ScoutSerial::serialWrite("Prelim bad trace. No harvesting position sent yet.", 50);

    if (scoutEmptyBufferState == 1)
        ScoutSerial::serialWrite("Bad trace. Did not empty buffer for more than 500ms.", 52);
    else
        ScoutSerial::serialWrite("Prelim good trace. Buffer always emptied.", 41);
}

#endif
