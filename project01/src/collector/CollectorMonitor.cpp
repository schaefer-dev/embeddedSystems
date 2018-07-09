#include "CollectorMonitor.h"

char CollectorMonitor::collectorCheckProximityState = 0;
char CollectorMonitor::collectorReactToPingState = 0;
char CollectorMonitor::collectorEmptyBufferState = 0;

bool CollectorMonitor::atHarvest = false;
bool CollectorMonitor::checkProximity = false;
unsigned long CollectorMonitor::lastBufferEmpty = 0;

const char messageBadCheckProximity[] = "Bad. Did not check proximity while not harvesting.";
const char messageBadPing[] = "Bad. Ignored 3 PING messages.";
const char messageBadEmptyBuffer[] = "Bad. Did not empty buffer.";

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
        Serial1.println("Prelim good. Always checked proximity when not harvesting.");

    if (collectorReactToPingState == 4)
        Serial1.println(messageBadPing);
    else
        Serial1.println("Prelim good. Never Ignored 3 consecutive PING messages.");

    unsigned long now = millis();
    if (now - lastBufferEmpty > 500 || collectorEmptyBufferState == 1) {
        collectorEmptyBufferState = 1;
        Serial1.println(messageBadEmptyBuffer);
    } else {
        Serial1.println("Prelim good. Buffer always emptied.");
    }
}

void CollectorMonitor::getStatus(char* status) {
    if (collectorCheckProximityState == 1) {
        status[0] = 'b';
        status[1] = 'b';
        status[2] = ',';
    } else {
        status[0] = 'p';
        status[1] = 'g';
        status[2] = ',';
    }
    if (collectorReactToPingState == 4) {
        status[3] = 'b';
        status[4] = 'b';
        status[5] = ',';
    } else {
        status[3] = 'p';
        status[4] = 'g';
        status[5] = ',';
    }
    unsigned long now = millis();
    if (now - lastBufferEmpty > 500 || collectorEmptyBufferState == 1) {
        collectorEmptyBufferState = 1;
        status[6] = 'b';
        status[7] = 'b';
        status[8] = ' ';
    } else {
        status[6] = 'p';
        status[7] = 'g';
        status[8] = ' ';
    }
}
