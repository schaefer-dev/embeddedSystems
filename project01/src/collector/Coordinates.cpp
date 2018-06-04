#include <stddef.h>
#include "Coordinates.h"

//constructor
Coordinate_queue::Coordinate_queue(){
    head = NULL;
}

//deconstructor
Coordinate_queue::~Coordinate_queue(){
    Coordinate_node *next_node = head;
    while (next_node){
        struct Coordinate_node *delete_node = next_node;
        next_node = next_node->next;
        delete delete_node;
    }
}

//append to queue
void Coordinate_queue::append(int x, int y){
    struct Coordinate_node *coordinate_node = new Coordinate_node();
    coordinate_node->x = x;
    coordinate_node->y = y;
    coordinate_node->next = NULL;
    if (head == NULL){
        head = coordinate_node;
        tail = coordinate_node;
    } else {
        tail->next = coordinate_node;
        tail = coordinate_node;
    }
}

//pop queue head
struct Coordinate_queue::Coordinate_node* Coordinate_queue::pop(){
    if (head == NULL){
        return NULL;
    }
    struct Coordinate_node *return_node = head;

    head = head->next;
    return return_node;
}
