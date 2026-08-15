#ifndef TASK_START_H
#define TASK_START_H
#include "stm32f4xx.h"
#include "config.h"
#include "task.h"


typedef  void (*PFTASK)(void *pMsg);
typedef struct
{
   PFTASK  pfTask;
   void *pMsg;    
}TASK_S;
typedef struct
{
    UINT16 period;
      TASK_S const*pGroup;
}TASK_GROUP_S;

typedef struct
{
    UINT8 flag;
      UINT16 cnt[8];
      UINT8 enable;
      TASK_GROUP_S const*pTaskList;
}TIME_SLICE_S;
extern TIME_SLICE_S MultiTask;
void initMultiTask(void);
void timeSlice(void);
void task_exec(void);

void SysTick_Init(UINT8 sysClk);


#endif
