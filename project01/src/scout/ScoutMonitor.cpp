#include "ScoutMonitor.h"

#ifdef SCOUT_MONITOR

#include "ScoutSerial.h"
#include "OrangutanTime.h"

char ScoutMonitor::scoutHarvestingState;
char ScoutMonitor::scoutReactToPingState;
char ScoutMonitor::scoutEmptyBufferState;
unsigned long ScoutMonitor::lastBufferEmpty;

char messageGoodSendHarvest[] = "Good trace. Sent at least one harvesting position.\n";
char messageBadPing[] = "Bad trace. Ignored 3 consecutive PING messages.\n";
char messageBadEmptyBuffer[] = "Bad trace. Did not empty buffer for more than 500ms.\n";

void ScoutMonitor::logSendHarvest() {
    if (scoutHarvestingState < 1) {
        scoutHarvestingState = 1;
        ScoutSerial::serialWrite(messageGoodSendHarvest, 51);
    }
}

void ScoutMonitor::logPingScout() {
    if (scoutReactToPingState < 4) {
        scoutReactToPingState++;
        if (scoutReactToPingState == 4) {
            ScoutSerial::serialWrite(messageBadPing, 48);
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
        ScoutSerial::serialWrite(messageBadEmptyBuffer, 53);
    }
    lastBufferEmpty = now;
}

void ScoutMonitor::verifyState() {
    if (scoutReactToPingState == 4)
        ScoutSerial::serialWrite(messageBadPing, 48);
    else
        ScoutSerial::serialWrite("Prelim good trace. Never Ignored 3 consecutive PING messages.\n", 62);

    if (scoutHarvestingState == 1)
        ScoutSerial::serialWrite(messageGoodSendHarvest, 51);
    else
        ScoutSerial::serialWrite("Prelim bad trace. No harvesting position sent yet.\n", 51);

    unsigned long now = millis();
    if (now - lastBufferEmpty > 500 || scoutEmptyBufferState == 1) {
        scoutEmptyBufferState = 1;
        ScoutSerial::serialWrite(messageBadEmptyBuffer, 53);
    } else {
        ScoutSerial::serialWrite("Prelim good trace. Buffer always emptied.\n", 42);
    }
}

#endif
