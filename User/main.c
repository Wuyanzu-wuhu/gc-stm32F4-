/**
 * main.c - STM32F407VGT6 工训赛主程序
 * 
 * 从 STM32F103RCT6 (F1) 移植至 STM32F407VGT6 (F4)
 * 
 * 引脚映射变更:
 *   HWT101: USART2(PA2/PA3) → UART4  (PC10/PC11)
 *   K230:   UART4(PC10/PC11)  → USART1 (PA9/PA10)
 *   MC:     USART1(PA9/PA10)  → USART3 (PB10/PB11)
 *   SaoM:   USART3(PB10/PB11) → USART6 (PC6/PC7)
 * 
 * 时钟: HSE 8MHz → PLL → 168MHz (F1: 72MHz)
 */
#include "stm32f4xx.h"
#include "HWT101.h"
#include "PID.h"
#include "Delay.h"
#include "math.h"
#include "control.h"
#include "task.h"
#include "task_start.h"
#include "MC.h"
#include "K230.h"
#include "SaoM.h"
#include "Servo.h"
#include "lcd.h"
extern  float motor_v[4] ; 

extern float vx_body;
extern  float vy_body;
extern float pid_input_error;


int main()
{ 
    count_TIM4_Init();           // 微秒延时定时器 (TIM4, 84MHz→1us)--
    Hwt101_Dma_Serial_Init();    // HWT101 姿态传感器 (UART4+PC10/PC11+DMA)
    MC_DMA_Serial_Init();        // 步进电机控制   (USART3+PB10/PB11+DMA)
    K230_Dma_Serial_Init();      // K230 视觉模块   (USART1+PA9/PA10+DMA)
    SaoM_Dma_Serial_Init();      // 扫码模块       (USART6+PC6/PC7+DMA)
    initMultiTask();             // 初始化多任务状态机
    SysTick_Init(168);           // SysTick 1ms (168MHz/8=21MHz, LOAD=21000)
	  Servo_CH1_Init();
   Servo_CH2_Init();
   Servo_CH3_Init();
	 LCD_Init();                            
   LCD_Fill(0, 0,320, 240, WHITE);
	  Scan_Function();
	//Servo_SmoothSetAngle_CH1(0,100);
 //Servo_SmoothSetAngle_CH2(0,100);
//	 Servo_SmoothSetAngle_CH3(130,1000);
//	ZDT_JueDui_Position_Control(0x02, behind, 50, 0, 880);
	//Delay_ms(2);
	//ZDT_JueDui_Position_Control(0x03, behind, 700, 150, 9500);

	 //Delay_s(1);
	//Servo_SetAngle_CH3(0);

    while(1)
    {       
       task_exec();             // 时间片轮转调度
    }
}
