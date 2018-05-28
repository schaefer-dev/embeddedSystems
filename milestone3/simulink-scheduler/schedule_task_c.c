#include "schedule_task_c.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

int tasklist_c[8] = { 0 };

void initialize_tasklist_c() {
    int i = 0;
    for (i = 0; i < 8; i++) {
        tasklist_c[i] = 0;
    }
	//tasklist[1] = 1;
}

void accept_task_c(int task) {
    tasklist_c[task] = 1; 
}

int next_task_c() {
	int i;	
	for(i=7; i>0; i--) {
		if (tasklist_c[i]==1) 
			return i;
	}
	return 0;
}

void remove_task_c(int task) {
	tasklist_c[task] = 0;
}
