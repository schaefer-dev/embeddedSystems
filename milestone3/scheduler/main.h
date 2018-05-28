#include "taskList.h"

void execute_task(struct task *task);
void interrupt_add_task(enum task_identifiers_collector task_identifier, struct task_list *task_list);
void schedule_next_task(struct task_list *task_list);
int main(int argc, const char * argv[]);
