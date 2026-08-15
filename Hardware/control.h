#ifndef __CONTROL_H
#define __CONTROL_H
#include "stm32f4xx.h"
#include "MC.h"

typedef enum
{ Scan_Qr,
    Grap1,
    Grap_and_put1,
    put1,
    Grap2,
    Grap_and_put2,
    put2,
    final
}DingChen_State_Type;

typedef enum
{ 
    Scan_Qr_S,
    Grap1_S,
  Grap_and_put1_S,
    put1_S,
    Grap2_S,
    Grap_and_put2_S,
    put2_S,
    final_S
}PID_State_Type;

typedef enum
{ Scan_Qr_Idle,
    Scan_Qr_Start,
    Scan_Qr_Success,
}Scan_Qr_State_Type;

typedef enum
{ Grap1_Idle,    
    Grap1_Start,
    Grap1_Visual_1,
    Grap1_Action_1,
    Grap1_Next_1,
    Grap1_Visual_2,
    Grap1_Action_2,
    Grap1_Next_2,
    Grap1_Visual_3,
    Grap1_Action_3,
    Grap1_Success,
}Grap1_State_Type;

typedef enum
{ 
    Grap_and_put1_Idle,
    Grap_and_put1_Start,
    Grap_and_put1_First_Move,
    Grap_and_put1_Visual_1,             
    Grap_and_put1_Action1,
    Grap_and_put1_Move1,
    Grap_and_put1_Next_1, 
    Grap_and_put1_Visual_2,  
    Grap_and_put1_Action2,
    Grap_and_put1_Move2,
    Grap_and_put1_Next_2,
    Grap_and_put1_Visual_3, 
    Grap_and_put1_Action3,
    Grap_and_put1_Move3,
    Grap_and_put1_Next_3,
    Grap_and_put1_Action4,
    Grap_and_put1_Move4,
    Grap_and_put1_Action5,
    Grap_and_put1_Move5,
    Grap_and_put1_Action6,
    Grap_and_put1_Success,
}Grap_and_put1_State_Type;

typedef enum
{ 
    put1_Idle,
    put1_Start,
    put1_First_Move,
    put1_Visual_1,
    put1_Action1,
    put1_Move1,
    put1_Next_1, 
    put1_Visual_2,
    put1_Action2,
    put1_Move2,
    put1_Next_2, 
    put1_Visual_3,
    put1_Action3,
    put1_Success,
}put1_State_Type;

typedef enum
{ 
    Grap2_Idle,    
    Grap2_Start,
    Grap2_Visual_1,
    Grap2_Action_1,
    Grap2_Next_1,
    Grap2_Visual_2,
    Grap2_Action_2,
    Grap2_Next_2,
    Grap2_Visual_3,
    Grap2_Action_3,
    Grap2_Success,
}Grap2_State_Type;

typedef enum
{
    Grap_and_put2_Idle,
    Grap_and_put2_Start,
    Grap_and_put2_First_Move,
    Grap_and_put2_Visual_1,             
    Grap_and_put2_Action1,
    Grap_and_put2_Move1,
    Grap_and_put2_Next_1, 
    Grap_and_put2_Visual_2,  
    Grap_and_put2_Action2,
    Grap_and_put2_Move2,
    Grap_and_put2_Next_2,
    Grap_and_put2_Visual_3, 
    Grap_and_put2_Action3,
    Grap_and_put2_Move3,
    Grap_and_put2_Next_3,
    Grap_and_put2_Action4,
    Grap_and_put2_Move4,
    Grap_and_put2_Action5,
    Grap_and_put2_Move5,
    Grap_and_put2_Action6,
    Grap_and_put2_Success,
}Grap_and_put2_State_Type;

typedef enum
{ 
    put2_Idle,
    put2_Start,
    put2_First_Move,
    put2_Visual_1,
    put2_Action1,
    put2_Move1,
    put2_Next_1, 
    put2_Visual_2,
    put2_Action2,
    put2_Move2,
    put2_Next_2, 
    put2_Visual_3,
    put2_Action3,
    put2_Success,
}put2_State_Type;

typedef struct {
      PID_Mode_Type type;
    float target_yaw;
    float vx1, vy1;
    float vx2, vy2;
    uint32_t t_acc, t_cru, t_dec;
} PID_Data_Type;

typedef struct {
  char type[3];  
} Scan_Qr_Data_Type;

typedef struct {
    int16_t x;      
    int16_t y;        
    uint8_t Wei_Tiao_Data_Completed_Flag;    
} Grap1_Data_Type;

typedef struct {
  char type[3];  
} Grap_and_put1_Data_Type;

typedef struct {
     char type[3];  
}put1_Data_Type;
typedef struct {
     char type[3];  
}Grap2_Data_Type;

typedef struct {
  char type[3];  
} Grap_and_put2_Data_Type;

typedef struct {
     char type[3];  
} put2_Data_Type;

//状态变量
extern DingChen_State_Type DingChen_State;
extern PID_State_Type PID_State;
extern Scan_Qr_State_Type Scan_Qr_State;
extern Grap1_State_Type Grap1_State;
extern Grap_and_put1_State_Type  Grap_and_put1_State;
extern put1_State_Type put1_State;
extern Grap2_State_Type  Grap2_State;
extern Grap_and_put2_State_Type Grap_and_put2_State;
extern put2_State_Type put2_State;

//数据变量
extern Scan_Qr_Data_Type Scan_Qr_Data;
extern Grap1_Data_Type Grap1_Data;
extern Grap_and_put1_Data_Type Grap_and_put1_Data;
extern put1_Data_Type put1_Data;
extern Grap2_Data_Type  Grap2_Data;
extern Grap_and_put2_Data_Type Grap_and_put2_Data;
extern put2_Data_Type put2_Data;

//完成标志位
extern uint8_t PID_Task_Completed_Flag;
extern uint8_t Special_Task_Completed_Flag;

/*系统节拍定时器变量*/
extern volatile uint32_t System_Tick_10ms;

void DingChen_State_Function(void *arg);
void PID_State_Function(void);
void Scan_Qr_State_Function(void);
void Grap1_State_Function(void);
void Grap_and_put1_State_Function(void);
void put1_State_Function(void);
void Grap2_State_Function(void);
void Grap_and_put2_State_Function(void);
void put2_State_Function(void);

#endif
