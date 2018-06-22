#include "CollectorMonitor.h"
#ifdef COLLECTOR_MONITOR
#include <Arduino.h>


char CollectorMonitor::collectorCheckProximityState = 0;
char CollectorMonitor::collectorReactToPingState = 0;
char CollectorMonitor::collectorEmptyBufferState = 0;

bool CollectorMonitor::atHarvest = false;
bool CollectorMonitor::checkProximity = false;
unsigned long CollectorMonitor::lastBufferEmpty = 0;

const char messageBadCheckProximity[] = "Bad trace. Did not check proximity while not harvesting.";
const char messageBadPing[] = "Bad trace. Ignored 3 consecutive PING messages.";
const char messageBadEmptyBuffer[] = "Bad trace. Did not empty buffer for more than 500ms.";

void CollectorMonitor::logAtHarvest(bool state) {
    atHarvest = state;

    if (collectorCheckProximityState == 0) {
        if (!(atHarvest || checkProximity)) {
            collectorCheckProximityState = 1;
            Serial1.println(messageBadCheckProximity);
        }
    }
}

void CollectorMonitor::logCheckProximity(bool state) {
    checkProximity = state;

    if (collectorCheckProximityState == 0) {
        if (!(atHarvest || checkProximity)) {
            collectorCheckProximityState = 1;
            Serial1.println(messageBadCheckProximity);
        }
    }
}

void CollectorMonitor::logPingCollector() {
    if (collectorReactToPingState < 4) {
        collectorReactToPingState++;
        if (collectorReactToPingState == 4) {
            Serial1.println(messageBadPing);
        }
    }
}

void CollectorMonitor::logPongCollector() {
    if (collectorReactToPingState < 4) {
        collectorReactToPingState = 0;
    }
}

void CollectorMonitor::emptyBuffer() {
    unsigned long now = millis();
    if (now - lastBufferEmpty > 500) {
        collectorEmptyBufferState = 1;
        Serial1.println(messageBadEmptyBuffer);
    }
    lastBufferEmpty = millis();
}

void CollectorMonitor::verifyState() {
    if (collectorCheckProximityState == 1)
        Serial1.println(messageBadCheckProximity);
    else
        Serial1.println("Prelim good trace. Always checked proximity when not harvesting.");

    if (collectorReactToPingState == 4)
        Serial1.println(messageBadPing);
    else
        Serial1.println("Prelim good trace. Never Ignored 3 consecutive PING messages.");

    unsigned long now = millis();
    if (now - lastBufferEmpty > 500 || collectorEmptyBufferState == 1) {
        collectorEmptyBufferState = 1;
        Serial1.println(messageBadEmptyBuffer);
    } else {
        Serial1.println("Prelim good trace. Buffer always emptied.");
    }
}

#endif
