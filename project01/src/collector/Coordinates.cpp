#include <stddef.h>
#include "Coordinates.h"

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

//pop queue head
struct CoordinateQueue::CoordinateNode* CoordinateQueue::pop(){
    if (head == nullptr){
        return nullptr;
    }
    struct CoordinateNode *returnNode = head;

    head = head->next;
    return returnNode;
}
