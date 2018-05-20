#include "taskList.h"
#include <stdlib.h>
#include <stdio.h>


int
main(int argc, const char * argv[])
  {
    printf("-------------------------------------- \n");
    printf("----------- STARTING TESTS ----------- \n");
    printf("-------------------------------------- \n");
    struct task_list *task_list = malloc (sizeof (struct task_list));
    initialize_task_list(task_list);


    /* test insertion in increasing priority order */
    struct task *task1 = create_task(TASK_CORRECT_THETA);
    accept_task (task1, task_list);

    struct task *task2 = create_task(TASK_CORRECT_THETA);
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
    struct task *task3 = create_task(TASK_GENERATE_NEW_GOAL);
    accept_task (task3, task_list);

    struct task *task4 = create_task(TASK_RECEIVE_SENSOR_READINGS);
    accept_task (task4, task_list);

    struct task *task5 = create_task(TASK_RECEIVE_SENSOR_READINGS);
    accept_task (task5, task_list);

    struct task *task6 = create_task(TASK_CORRECT_THETA);
    accept_task (task6, task_list);

    compare_task = pop_task(task_list);
    if (compare_task != task3)
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
  }
