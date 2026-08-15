#ifndef __MC_H
#define __MC_H

#include <stdio.h>
#include <stdlib.h>
#define PI 3.14159265f
#define CHASSIS_RADIUS 0.15f   // 麦轮底盘半径(米)，决定W的权重，需根据实际调参
#define SPEED_RATIO    10.0f   // 速度到步进电机脉冲的换算比例，需根据实际调参


#define SC_IDLE     0
#define SC_ACCEL    1
#define SC_CRUISE   2
#define SC_DECEL    3
#define SC_GUIWEI   4   // 平移结束后的硬归位
#define SC_TURNING  5   // 纯旋转
//下面是状态机内部调用函数会用到状态（如PID的两个模式）
typedef enum
{ Straight_and_Angle,
    Angle
}PID_Mode_Type;
extern PID_Mode_Type PID_Mode;
typedef enum
{straight,      //0是前进
 behind
}DJ_Direction_Type;
extern DJ_Direction_Type DJ_Direction;
extern float motor_v[4] ;  
extern uint8_t g_sc_state; 

void MC_DMA_Serial_Init(void);
void Serial_SendHex_DMA(uint8_t *Data, uint16_t Length);
void ZDT_Position_Control(uint8_t Addr,DJ_Direction_Type dir,uint16_t speed,uint32_t clk);
void ZDT_JueDui_Position_Control(uint8_t Addr, DJ_Direction_Type dir, uint16_t speed, uint8_t acc, uint32_t clk);
void ZDT_Speed_Control(uint8_t Addr,DJ_Direction_Type dir,uint16_t speed);
void ZDT_DJ_TongBu_Control(void);
void ZDT_Send_DJ_DaoWei_Control(uint8_t Addr);
uint8_t ZDT_Get_DJ_DaoWei_Flag(void);
uint8_t ZDT_Get_DJ_DaoWei_Sum_Flag(void);
uint8_t ZDT_Get_DJ_HuiLing_Flag(void);


void Chassis_Core(PID_Mode_Type mode, float target_yaw, 
                  float vx1, float vy1, float vx2, float vy2,
                  uint32_t t_acc, uint32_t t_cru, uint32_t t_dec, 
                  uint8_t start_flag);




#endif
