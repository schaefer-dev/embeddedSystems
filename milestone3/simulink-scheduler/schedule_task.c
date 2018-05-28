#include "schedule_task.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

int tasklist[8] = { 0 };

void initialize_tasklist() {
	tasklist[1] = 1;
}

void accept_task(int task) {
    tasklist[task] = 1; 
}

int next_task() {
	int i;	
	for(i=7; i>0; i--) {
		if (tasklist[i]==1) 
			return i;
	}
	return 0;
}

void remove_task(int task) {
	tasklist[task] = 0;
}
