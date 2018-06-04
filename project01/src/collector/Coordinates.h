#include <stddef.h>

#ifndef Coordinate_queue_H
#define Coordinate_queue_H

class CoordinateQueue
{
    // internal structure for coordinate nodes
    struct CoordinateNode
    {
        public:
        int x;
        int y;
        struct CoordinateNode *next;
    };

public:
    //constructor
    CoordinateQueue();

    //deconstructor
    ~CoordinateQueue();

    void append(int x, int y);

    struct CoordinateNode* pop();


private:
    struct CoordinateNode *head;
    struct CoordinateNode *tail;
};

#endif