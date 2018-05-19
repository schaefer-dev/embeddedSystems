#include "taskList.h"
#include <stdlib.h>
#include <stdio.h>


int
main(int argc, const char * argv[])
  {
    struct task_list *task_list = malloc (sizeof (struct task_list));
    initialize_task_list(task_list);

    struct task *task = create_task(TASK_CORRECT_THETA);
    insert_task_sorted(task_list, task);

    struct task *compare_task = get_list_head(task_list);

    if (compare_task != task)
      printf("WOOPS, something went wrong...\n");
    else
      printf("Test passed!\n");
  }
