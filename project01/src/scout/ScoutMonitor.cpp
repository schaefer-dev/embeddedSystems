#include "ScoutMonitor.h"

#ifdef SCOUT_MONITOR

#include "ScoutSerial.h"
#include "OrangutanTime.h"

char ScoutMonitor::scoutHarvestingState;
char ScoutMonitor::scoutReactToPingState;
char ScoutMonitor::scoutEmptyBufferState;
unsigned long ScoutMonitor::lastBufferEmpty;

char messageGoodSendHarvest[] = "Good. Harvesting position sent.\n";
char messageBadPing[] = "Bad. Ignored 3 PINGs.\n";
char messageBadEmptyBuffer[] = "Bad. Buffer not emptied.\n";

void ScoutMonitor::logSendHarvest() {
    if (scoutHarvestingState < 1) {
        scoutHarvestingState = 1;
        ScoutSerial::serialWrite(messageGoodSendHarvest, 32);
    }
}

void ScoutMonitor::logPingScout() {
    if (scoutReactToPingState < 4) {
        scoutReactToPingState++;
        if (scoutReactToPingState == 4) {
            ScoutSerial::serialWrite(messageBadPing, 22);
        }
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
        ScoutSerial::serialWrite(messageBadEmptyBuffer, 25);
    }
    lastBufferEmpty = now;
}

void ScoutMonitor::verifyState() {
    if (scoutReactToPingState == 4)
        ScoutSerial::serialWrite(messageBadPing, 22);
    else
        ScoutSerial::serialWrite("Prelim good. Pings not ignored.\n", 32);

    if (scoutHarvestingState == 1)
        ScoutSerial::serialWrite(messageGoodSendHarvest, 32);
    else
        ScoutSerial::serialWrite("Prelim bad. No harvesting position sent yet.\n", 45);

    unsigned long now = millis();
    if (now - lastBufferEmpty > 500 || scoutEmptyBufferState == 1) {
        scoutEmptyBufferState = 1;
        ScoutSerial::serialWrite(messageBadEmptyBuffer, 22);
    } else {
        ScoutSerial::serialWrite("Prelim good. Buffer emptied.\n", 29);
    }
}

#endif
