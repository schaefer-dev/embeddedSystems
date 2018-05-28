#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


/* lower task identifier -> lower priority
   New tasks are added here with INCREASING PRIORITY */
enum task_identifiers_collector
  {
    COLLECTOR_CORRECT_THETA = 001,
    COLLECTOR_GENERATE_NEW_GOAL = 002,
    COLLECTOR_RECEIVE_SENSOR_READINGS = 003,
    COLLECTOR_HANDLE_HIGH_SENSOR_READINGS = 004,
    COLLECTOR_RECEIVE_HARVESTING_POSITION = 005,
    COLLECTOR_HANDLE_REFEREE_UPDATE = 006,
  };

/* lower task identifier -> lower priority
   New tasks are added here with INCREASING PRIORITY */
enum task_identifiers_scout
  {
    SCOUT_ORIENTATION_AND_DRIVE = 001,
    SCOUT_GENERATE_NEW_GOAL = 002,
    SCOUT_COMMUNICATE_OWN_POSITION = 003,
    SCOUT_RECEIVE_SENSOR_READINGS = 004,
    SCOUT_HANDLE_HIGH_SENSOR_READINGS = 005,
    SCOUT_COMMUNICATE_HARVEST_COORDINATES = 006,
    SCOUT_HANDLE_REFEREE_UPDATE = 007,
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
    enum task_identifiers_collector task_identifier;
    short priority;
  };


void initialize_task_list (struct task_list*);
struct task* create_task (enum task_identifiers_collector);
void accept_task (struct task*, struct task_list*);
void free_task (struct task*);
struct task* pop_task (struct task_list*);
struct task* schedule (struct task_list*);
