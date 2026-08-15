#include "task.h"
#include "control.h"
const TASK_S  taskGroup0[]=
{
     
     {DingChen_State_Function,NULL},
};

const TASK_S  taskGroup1[]=
{
     
         {NULL,NULL},         
};
const TASK_S  taskGroup2[]=
{
     {NULL,NULL},
};
const TASK_S  taskGroup3[]=
{
     {NULL,NULL},
};

const TASK_GROUP_S TaskList[]=
{
     {10, taskGroup0},
         {30,taskGroup1},
         {40,taskGroup2},
         {50,taskGroup3},
}; 

UINT8 taskGroupNum(void)
{
    return sizeof(TaskList)/sizeof(TASK_GROUP_S);
}

void initTaskList(void)
{
     MultiTask.pTaskList = TaskList;
}
