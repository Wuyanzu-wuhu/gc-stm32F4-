#include "task_start.h"
#include "control.h"
#include "Servo.h"
TIME_SLICE_S  MultiTask;

void initMultiTask(void)
{
      UINT8 i;
      UINT8 num = taskGroupNum();
    MultiTask.flag = 0x00;
      MultiTask.enable = ~0x00;
      for(i=0;i<num;i++)
      {
        MultiTask.cnt[i] = 0x00;
        }
      initTaskList();
}

void timeSlice(void)
{
    UINT8 i;
      UINT8 num = taskGroupNum();
      for(i=0;i<num;i++)
    {
             if(MultiTask.enable & (1<<i))
         {
                     MultiTask.cnt[i]++;
                       if(MultiTask.cnt[i] >= MultiTask.pTaskList[i].period)
             {
                             MultiTask.flag |= (1<<i);
                               MultiTask.cnt[i] = 0x00;
                         }
                 }
        
        }
}

void task_exec(void)
{
     PFTASK p;
   UINT8 i;
     UINT8 j;
     UINT8 num = taskGroupNum();
     if(MultiTask.flag)
   {
         for(i=0;i<num;i++)
       {
                   
                 if(MultiTask.flag & (1 << i))
           {
                           j = 0x00;
                           MultiTask.flag &= ~(1<<i); 
                           p = MultiTask.pTaskList[i].pGroup[j].pfTask;
                         while(p != NULL)
               {
                                p(MultiTask.pTaskList[i].pGroup[j].pMsg);
                                  j++;
                                  p = MultiTask.pTaskList[i].pGroup[j].pfTask;
                             }
                     }
             }
     }
}



void SysTick_Init(UINT8 sysClk)
{
      SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); 
      SysTick->LOAD = ((UINT16)sysClk * 1000)/8;  // F4: sysClk=168 → LOAD=21000 → 1ms
      SysTick->VAL=0x00; 
      SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;
      SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk ;                   
}

void SysTick_Handler(void)
{
    
    static uint8_t div10 = 0;   // 10ms 分频计数器
    int temp = SysTick->CTRL;
    
    timeSlice();                // 任务调度心跳，保持 1ms
    
    // 每 10ms 给 MC.c 的 S 曲线提供时基
    if (++div10 >= 10) {
        div10 = 0;
        System_Tick_10ms++;
			Servo_SmoothUpdate();
    }
      
}
