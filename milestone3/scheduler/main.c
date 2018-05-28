#include "taskList.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#define TESTING

void
execute_task(struct task *task)
  {
    switch(task->task_identifier) {
      case COLLECTOR_CORRECT_THETA:
          usleep(500 * 1000);
          printf("Task finished execution after 500ms\n");
          break;
      case COLLECTOR_RECEIVE_SENSOR_READINGS:
          usleep(200 * 1000);
          printf("Task finished execution after 200ms\n");
          break;
      case COLLECTOR_HANDLE_HIGH_SENSOR_READINGS:
          usleep(1000 * 1000);
          printf("Task finished execution after 1000ms\n");
          break;
      case COLLECTOR_GENERATE_NEW_GOAL:
          usleep(2000 * 1000);
          printf("Task finished execution after 2000ms\n");
          break;
      case COLLECTOR_HANDLE_REFEREE_UPDATE:
          usleep(300 * 1000);
          printf("Task finished execution after 300ms\n");
          break;
      case COLLECTOR_RECEIVE_HARVESTING_POSITION:
          usleep(200 * 1000);
          printf("Task finished execution after 200ms\n");
          break;
    }
  }

void
interrupt_add_task(enum task_identifiers_collector task_identifier, struct task_list *task_list)
  {
    struct task *task = create_task(task_identifier);
    accept_task(task,task_list);
    printf("interrupt -> task with id %i was added\n", task->task_identifier);
    usleep(10 * 1000);
  }

void
schedule_next_task(struct task_list *task_list)
  {
    struct task *next_task = schedule(task_list);
    printf("scheduling called -> task with id %i was scheduled\n", next_task->task_identifier);
    if (next_task->task_identifier != COLLECTOR_CORRECT_THETA){
      pop_task(task_list);
      execute_task(next_task);
      free(next_task);
    } else {
      execute_task(next_task);
    }
  }



int
main(int argc, const char * argv[])
  {
    struct task_list *task_list = malloc (sizeof (struct task_list));
    initialize_task_list(task_list);
    #ifdef TESTING
    printf("-------------------------------------- \n");
    printf("----------- STARTING TESTS ----------- \n");
    printf("-------------------------------------- \n");

    /* test insertion in increasing priority order */
    struct task *task1 = create_task(COLLECTOR_CORRECT_THETA);
    accept_task (task1, task_list);

    struct task *task2 = create_task(COLLECTOR_CORRECT_THETA);
    printf("creating second task worked\n");

    accept_task (task2, task_list);

    printf("inserting second task worked\n");

    int i = 2;

    while (i < 8)
      {
        struct task *task_loop = create_task(i);
        accept_task (task_loop, task_list);
        i += 1;
      }

    int test_counter = 1;
    int debug_iter = 7;
    while (debug_iter >= 2)
      {
        struct task *compare_task = pop_task(task_list);
        if (compare_task->priority != debug_iter)
          printf("Test %i FAILED!\n", test_counter);
        else
          printf("Test %i passed!\n", test_counter);

        free_task(compare_task);
        debug_iter -= 1;
        test_counter += 1;
      }

    struct task *compare_task = pop_task(task_list);
    if (compare_task != task1)
      printf("Test %i FAILED!\n", test_counter);
    else
      printf("Test %i passed!\n", test_counter);

    free_task(compare_task);
    test_counter += 1;

    compare_task = pop_task(task_list);
    if (compare_task != task2)
      printf("Test %i FAILED!\n", test_counter);
    else
      printf("Test %i passed!\n", test_counter);

    free_task(compare_task);
    test_counter += 1;

    /* test insertion in decreasing priority order */
    struct task *task3 = create_task(COLLECTOR_GENERATE_NEW_GOAL);
    accept_task (task3, task_list);

    struct task *task4 = create_task(COLLECTOR_RECEIVE_SENSOR_READINGS);
    accept_task (task4, task_list);

    struct task *task5 = create_task(COLLECTOR_RECEIVE_SENSOR_READINGS);
    accept_task (task5, task_list);

    struct task *task6 = create_task(COLLECTOR_CORRECT_THETA);
    accept_task (task6, task_list);

    compare_task = pop_task(task_list);
    if (compare_task != task4)
      printf("Test %i FAILED!\n", test_counter);
    else
      printf("Test %i passed!\n", test_counter);

    free_task(compare_task);
    test_counter += 1;

    compare_task = pop_task(task_list);
    if (compare_task->priority != 3)
      printf("Test %i FAILED!\n", test_counter);
    else
      printf("Test %i passed!\n", test_counter);

    free_task(compare_task);
    test_counter += 1;

    compare_task = pop_task(task_list);
    if (compare_task->priority != 2)
      printf("Test %i FAILED!\n", test_counter);
    else
      printf("Test %i passed!\n", test_counter);

    free_task(compare_task);
    test_counter += 1;


    compare_task = pop_task(task_list);
    if (compare_task != task6)
      printf("Test %i FAILED!\n", test_counter);
    else
      printf("Test %i passed!\n", test_counter);

    free_task(compare_task);
    test_counter += 1;

    printf("-------------------------------------- \n");
    printf("------------ ENDING TESTS ------------ \n");
    printf("-------------------------------------- \n");
    #endif

    /* add task which will always be contained to list */
    struct task *default_task = create_task(COLLECTOR_CORRECT_THETA);
    accept_task (default_task, task_list);

    schedule_next_task(task_list);
    interrupt_add_task(COLLECTOR_GENERATE_NEW_GOAL, task_list);
    interrupt_add_task(COLLECTOR_RECEIVE_SENSOR_READINGS, task_list);
    schedule_next_task(task_list);
    schedule_next_task(task_list);
    schedule_next_task(task_list);
    schedule_next_task(task_list);
    schedule_next_task(task_list);
    interrupt_add_task(COLLECTOR_RECEIVE_SENSOR_READINGS, task_list);
    interrupt_add_task(COLLECTOR_HANDLE_REFEREE_UPDATE, task_list);
    schedule_next_task(task_list);
    schedule_next_task(task_list);
    schedule_next_task(task_list);
    schedule_next_task(task_list);
  }
