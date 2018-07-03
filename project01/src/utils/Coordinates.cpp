#include "Coordinates.h"
#include "math.h"
#include "../scout/ScoutSerial.h"
#include <stdlib.h>

//constructor
CoordinateQueue::CoordinateQueue(){
    head = nullptr;
}

//deconstructor
CoordinateQueue::~CoordinateQueue(){
    CoordinateNode *nextNode = head;
    while (nextNode){
        struct CoordinateNode *deleteNode = nextNode;
        nextNode = nextNode->next;
        delete deleteNode;
    }
}

bool CoordinateQueue::isEmpty(){
    if (head == nullptr)
        return true;
    return false;
}

//append to queue
void CoordinateQueue::append(int x, int y){
    ScoutSerial::serialWrite("appending to queue\n", 19);
    ScoutSerial::serialWriteInt(x);
    ScoutSerial::serialWriteInt(y);
    //CoordinateNode *coordinateNode = (CoordinateNode * )malloc(sizeof(CoordinateNode));
    auto *coordinateNode = new CoordinateNode();
    coordinateNode->x = x;
    coordinateNode->y = y;
    coordinateNode->next = nullptr;
    if (head == nullptr){
        head = coordinateNode;
        tail = coordinateNode;
    } else {
        tail->next = coordinateNode;
        tail = coordinateNode;
    }
}

/**
 * Returns the closest destination based on euclidean distance.
 * If multiple destinations have the same distance, chooses by FIFO
 * @param currentX
 * @param currentY
 * @return Pointer to the destination CoordinateNode
 */
struct CoordinateQueue::CoordinateNode* CoordinateQueue::pop(float currentXfloat, float currentYfloat){
    ScoutSerial::serialWrite("popping from queue\n", 19);
    if (head == nullptr){
        return nullptr;
    }

    int currentX = (int) currentXfloat;
    int currentY = (int) currentYfloat;

    struct CoordinateNode *bestNodePrev = nullptr;
    struct CoordinateNode *bestNode = nullptr;

    float bestDistance = INFINITY;

    auto *nextNode = head;
    auto *prevNode = nextNode;
    while (nextNode){
        float distance = sqrt(pow(currentX - nextNode->x, 2) + pow(currentY - nextNode->y, 2));
        if (distance <= bestDistance) {
            bestDistance = distance;
            bestNode = nextNode;
            bestNodePrev = prevNode;
        }
        prevNode = nextNode;
        nextNode = nextNode->next;
    }
    if (bestNode != nullptr) {
        if (bestNodePrev == bestNode) {
            /* if best element equals head */
            head = bestNode->next;
            if (tail == bestNode)
                tail = nullptr;
        } else {
            bestNodePrev->next = bestNode->next;
        }
    }
    return bestNode;
}

/**
 * Returns the closest destination based on euclidean distance.
 * If multiple destinations have the same distance, chooses by FIFO
 * @param currentX
 * @param currentY
 * @return Pointer to the destination CoordinateNode
 */
void CoordinateQueue::getHeadCoordinates(float currentXfloat, float currentYfloat, int* returnArray){
    ScoutSerial::serialWrite("popping from queue\n", 19);

    if (head == nullptr)
        return;

    returnArray[0] = head->x;
    returnArray[1] = head->y;

}