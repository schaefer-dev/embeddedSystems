#include "taskList.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* TODO: import malloc/free */


void
initialize_task_list (struct task_list *task_list) {
  ASSERT(task_list != NULL);
  task_list->is_empty = true;
  task_list->head = NULL;
  task_list->tail = NULL;
}

struct task*
create_task (enum task_identifiers task_identifier)
{
  /* TODO implement deadline,priority,etc. for specific task identifiers */
  struct task *task = malloc (sizeof (struct task));
  task->task_identifier = task_identifier;
  task->next = NULL;
  task->prev = NULL;

  /* TODO change these example priorities */
  if (task_identifier > 5)
  {
    task->priority = 2;
  } else {
    task->priority = 1;
  }
}

void
insert_task_sorted (struct task_list *task_list, struct task *task)
{
  ASSERT(task_list != NULL);
  ASSERT(task != NULL);
  ASSERT(task->prev == NULL && task->next == NULL);
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
    while (task_iterator != NULL && task_iterator->priority > task->priority)
    {
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
      task->prev = task_iterator->prev;
      task_iterator->prev->next = task;
      task_iterator->prev = task;
    }
  }
}

/* removes the head from the list and returns it, returns NULL if empty*/
struct task*
pop_task (struct task_list *task_list){
  ASSERT(task_list != NULL);
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
get_list_head (struct task_list *task_list)
{
  ASSERT(task_list != NULL);
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
  ASSERT(task != NULL);
  return task->next;
}

struct task*
previous_task (struct task *task)
{
  ASSERT(task != NULL);
  return task->prev;
}
