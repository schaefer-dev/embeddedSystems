#ifndef Coordinate_queue_H
#define Coordinate_queue_H

class CoordinateQueue
{

public:

    // internal structure for coordinate nodes
    struct CoordinateNode
    {
    public:
        int x;
        int y;
        struct CoordinateNode *next;
    };

    //constructor
    CoordinateQueue();

    //deconstructor
    ~CoordinateQueue();

    void append(int x, int y);

    struct CoordinateNode* pop(float, float);


private:
    struct CoordinateNode *head;
    struct CoordinateNode *tail;
};

#endif