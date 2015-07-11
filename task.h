#ifndef _TASK_H
#define _TASK_H

// @summary Adds the specified routine to be executed at task level
// @returns 1 indicates the task was added, 0 indicates it was not
uint8_t task_add(void (*pfn)(void*, void*), void* pContext1, void* pContext2);

// @summary Dispatches any available tasks
void task_run(void);

// @summary Waits for a task to be added
void task_wait(void);

#endif
