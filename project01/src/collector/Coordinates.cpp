#include "Coordinates.h"
#include "math.h"
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

//append to queue
void CoordinateQueue::append(int x, int y){
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
struct CoordinateQueue::CoordinateNode* CoordinateQueue::pop(float currentX, float currentY){
    if (head == nullptr){
        return nullptr;
    }

    struct CoordinateNode *bestNode = nullptr;
    float bestDistance = INFINITY;

    auto *nextNode = head;
    while (nextNode){
        float distance = sqrt(pow(currentX - nextNode->x, 2) + pow(currentY - nextNode->y, 2));
        if (distance <= bestDistance) {
            bestDistance = distance;
            bestNode = nextNode;
        }
        nextNode = nextNode->next;
    }

    return bestNode;
}