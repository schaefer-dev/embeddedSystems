#include <stddef.h>

#ifndef Coordinate_queue_H
#define Coordinate_queue_H

class Coordinate_queue
{
    // internal structure for coordinate nodes
    struct Coordinate_node
    {
        public:
        int x;
        int y;
        struct Coordinate_node *next;
    };

public:
    //constructor
    Coordinate_queue();

    //deconstructor
    ~Coordinate_queue();

    void append(int x, int y);

    struct Coordinate_node* pop();


private:
    struct Coordinate_node *head;
    struct Coordinate_node *tail;
};

#endif