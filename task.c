#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "task.h"
#include <stdlib.h>

typedef struct _task_entry {
	union {
		void (*pfn)(void*, void*);
		uint32_t pfnValue;
	};
	void* pContext1;
	void* pContext2;
} task_entry;

static task_entry tasks[32] = {};

uint8_t task_add(void (*pfn)(void*, void*), void* pContext1, void* pContext2) {
	for(int i = 0; i < 32; i++) {
		uint32_t pfnValue = __LDREXW(&tasks[i].pfnValue);
		if(pfnValue)
			// Already claimed
			continue;
		
		// Try to claim the slot:
		if(__STREXW((uint32_t)pfn, &tasks[i].pfnValue))
			// Failed, someone else interrupted us, go to the next slot
			continue;

		// Fill out the context fields.  We don't have to synchronize, here, because
		// task_run can only be performed at passive level.
		tasks[i].pContext1 = pContext1;
		tasks[i].pContext2 = pContext2;
		__SEV();
		return 1;
	}
	return 0;
}

void task_run(void) {
	for(int i = 0; i < 32; i++) {
		if(!tasks[i].pfn)
			// No further tasks to be run
			return;
		
		// Clear the task, we have executed it
		tasks[i].pfn(tasks[i].pContext1, tasks[i].pContext2);
		tasks[i].pfn = NULL;
	}
}

void task_wait(void) {
	__WFE();
}
