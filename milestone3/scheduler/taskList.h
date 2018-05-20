#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


/* lower task identifier -> lower priority
   New tasks are added here with INCREASING PRIORITY */
enum task_identifiers
  {
    TASK_CORRECT_THETA = 001,
    TASK_RECEIVE_SENSOR_READINGS = 002,
    TASK_HANDLE_HIGH_SENSOR_READINGS = 003,
    TASK_GENERATE_NEW_GOAL = 004,
    TASK_COMMUNICATE_CURRENT_POSITION = 005,
    TASK_HANDLE_REFEREE_UPDATE = 006,
    TASK_RECEIVE_HARVESTING_POSITION = 007,
    /* TODO to be continued ... */
  };

/* Struct to maintain references to the entire List */
struct task_list
  {
    struct task *head;
    struct task *tail;
    bool is_empty;
  };

/* element contained in taskList, doubly linked
   TODO: priority will be = task_identifier */
struct task
  {
    struct task *prev;
    struct task *next;
    enum task_identifiers task_identifier;
    short priority;
  };


void initialize_task_list (struct task_list*);
struct task* create_task (enum task_identifiers);
void accept_task (struct task*, struct task_list*);
void free_task (struct task*);
struct task* pop_task (struct task_list*);
struct task* schedule (struct task_list*);
