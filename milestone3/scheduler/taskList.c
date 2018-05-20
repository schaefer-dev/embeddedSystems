#include "taskList.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>


/* DEBUG Macro enables Debugging mode, which includes error 'catching' prints
  and various additional output, which should be disabled once fully working to
  ensure maximum performance */
#define DEBUG

void
initialize_task_list (struct task_list *task_list)
  {
    #ifdef DEBUG
    if(task_list == NULL)
      printf("initilize_task_list was called with parameter NULL\n");
    #endif

    task_list->is_empty = true;
    task_list->head = NULL;
    task_list->tail = NULL;
  }

struct task*
create_task (enum task_identifiers task_identifier)
  {
    struct task *task = malloc (sizeof (struct task));
    task->task_identifier = task_identifier;
    task->next = NULL;
    task->prev = NULL;

    /* TODO maybe change these priorities later */
    task->priority = task->task_identifier;

    return task;
  }

void
accept_task (struct task *task, struct task_list *task_list)
  {
    #ifdef DEBUG
    if(task_list == NULL || task == NULL)
      printf("insert_task_sorted was called with parameter NULL\n");
    if(task->prev != NULL || task->next != NULL)
      printf("insert_task_sorted task already has next or previous attribute set\n");
    #endif

    if (task_list->is_empty)
      {
        task_list->head = task;
        task_list->tail = task;
        task->next = NULL;
        task->prev = NULL;
        task_list->is_empty = false;
        return;
      } else {
        struct task *task_iterator = task_list->head;

        /* search first list element which has smaller probability, or go to last list element, if there is no such element contained */
        while (task_iterator != NULL && task_iterator->priority >= task->priority)
          {

            if (task_iterator->priority == task->priority)
              printf("WARNING: Task with ID %i was inserted despite being contained already!\n", task->task_identifier);

            task_iterator = task_iterator->next;
          }

        /* at this point task_iterator on first element with smaller or equal priority, or on null if no such element contained */
        if (task_iterator == NULL)
          {
            task_list->tail->next = task;
            task->prev = task_list->tail;
            task_list->tail = task;
            return;
          } else {
            task->next = task_iterator;

            if (task_iterator->prev == NULL)
              {
                /* inserted at front sets new head value of list */
                task_list->head = task;
              } else {
                task_iterator->prev->next = task;
              }
            task->prev = task_iterator->prev;
            task_iterator->prev = task;
          }
      }
  }

/* removes the head from the list and returns it, returns NULL if empty*/
struct task*
pop_task (struct task_list *task_list)
  {
    #ifdef DEBUG
    if(task_list == NULL)
      printf("pop_task was called with parameter NULL\n");
    #endif

    /* handle case for empty list */
    if (task_list->is_empty)
      {
        return NULL;
      }
    /* handle case for only 1 element contained */
    if (task_list->head->next == NULL)
      {
        struct task *previous_head = task_list->head;
        task_list->head = NULL;
        task_list->tail = NULL;
        task_list->is_empty = true;

        previous_head->next = NULL;
        return previous_head;
      }
    /* TODO: check if this handling of previous head works and the ressources are freed correctly */
    struct task *previous_head = task_list->head;
    previous_head->next->prev = NULL;
    task_list->head = previous_head->next;

    previous_head->next = NULL;
    return previous_head;
  }

/* returns first task in the queue which should be the next task to be worked on */
struct task*
schedule (struct task_list *task_list)
  {
    #ifdef DEBUG
    if (task_list == NULL)
      printf("get_list_head was called with parameter NULL\n");
    #endif

    return task_list->head;
  }

void
free_task (struct task *task)
  {
    /* TODO: free all structures that have been allocated for task */
    free(task);
    return;
  }

struct task*
next_task (struct task *task)
  {
    #ifdef DEBUG
    if (task == NULL)
      printf("next_task was called with parameter NULL\n");
    #endif

    return task->next;
  }

struct task*
previous_task (struct task *task)
  {
    #ifdef DEBUG
    if (task == NULL)
      printf("previous_task was called with parameter NULL\n");
    #endif

    return task->prev;
  }
