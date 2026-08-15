#include "stm32f4xx.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "MC.h"
#include "math.h"
#include "PID.h"
#include "Delay.h"
#include "K230.h"
#include "HWT101.h"

/* ========== 引脚映射 (F4) ==========
   USART2_TX -> PA2   (AF7)
   USART2_RX -> PA3  (AF7)
   RX DMA    -> DMA1_Stream5_Channel4
   TX DMA    -> DMA1_Stream6_Channel4
   ================================== */

#define DMA_RX_BUF_SIZE 128
uint8_t DMA2_RxBuf[DMA_RX_BUF_SIZE]; // USART1 专用 DMA 接收缓存


/* ========== 全局视觉数据 ========== */
volatile int16_t  g_visual_dx        = 0;
volatile int16_t  g_visual_dy        = 0;
volatile uint8_t  g_visual_stop_flag = 0;

/* ========== 帧解析变量 (中断内部使用) ========== */
/* 帧格式: [0xAA][0x55][dx_h][dx_l][dy_h][dy_l][0x0D] 共7字节 */
static uint8_t RxState   = 0;
static uint8_t pRxPacket = 0;
static uint8_t rx_buffer[7];

// ====== 初始化 ======

void K230_Dma_Serial_Init(void)
{
    /* ---- 1. 开时钟 (F4: GPIO/DMA在AHB1, USART1在APB2) ---- */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* ---- 2. GPIO -- PA9(TX) / PA10(RX) ---- */
    GPIO_InitTypeDef GPIO_InitStructure;
    // PA2 - USART1_TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

    // PA3 - USART1_RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    /* ---- 3. USART1 参数 ---- */
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);

    /* ---- 4. DMA 接收 (DMA2_Stream2_Channel4) ---- */
    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA1_Stream5);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DMA2_RxBuf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = DMA_RX_BUF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream5, &DMA_InitStructure);

    /* ---- 5. DMA 发送 (DMA2_Stream7_Channel4) ---- */
    DMA_DeInit(DMA1_Stream6);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = 0;       // 发送时指定
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = 0;            // 发送时指定
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream6, &DMA_InitStructure);

    /* ---- 6. 中断 ---- */
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* ---- 7. 使能 ---- */
    USART_Cmd(USART2, ENABLE);
    USART_DMACmd(USART2, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Stream5, ENABLE); // 开启DMA接收
}

void K230_Dma_SendHex(uint8_t *Data, uint16_t Length)
{
    // 步骤1：如果 DMA 正在运行，等待上一次传输完成
    if((DMA1_Stream6->CR & DMA_SxCR_EN) != RESET)
    {
        while(DMA_GetFlagStatus(DMA1_Stream6, DMA_FLAG_TCIF6) == RESET);
    }

    // 步骤2：软件关闭 DMA 通道
    DMA_Cmd(DMA1_Stream6, DISABLE);

    // 步骤3：等待硬件真正关闭
    while((DMA1_Stream6->CR & DMA_SxCR_EN) != RESET);

    // 步骤4：清除 DMA 所有标志位
    DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_FEIF6);

    // 步骤5：重新配置内存地址和传输数量
    DMA1_Stream6->M0AR = (uint32_t)Data;
    DMA1_Stream6->NDTR = Length;

    // 步骤6：启动 DMA
    DMA_Cmd(DMA1_Stream6, ENABLE);
}

/* ===== 视觉帧特殊值 ===== */
#define VISION_ARRIVED_VAL  32767   /* dx=32767, dy=32767 表示"到位了" */

void USART2_IRQHandler(void)
{
    if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET) {
        volatile uint8_t d = USART2->DR;
        (void)d;
    }
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        volatile uint32_t t = USART2->SR;
        t = USART2->DR;
        (void)t;
        uint16_t rx_len = DMA_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream5);

        DMA_Cmd(DMA1_Stream5, DISABLE);
			    while (DMA1_Stream5->CR & DMA_SxCR_EN);   // 必须等待！
			   DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5          //这个最关键了
                    | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
        DMA_SetCurrDataCounter(DMA1_Stream5, DMA_RX_BUF_SIZE);
        DMA_Cmd(DMA1_Stream5, ENABLE);
       /* 逐字节喂入状态机 */
        for (uint16_t i = 0; i < rx_len; i++)
        {
            uint8_t Rxdata = DMA2_RxBuf[i];

            if (RxState == 0)
            {
                /* 等帧头第1字节 0xAA */
                if (Rxdata == 0xAA)
                {
                    rx_buffer[0] = Rxdata;
                    pRxPacket = 1;
                    RxState = 1;
                }
            }
            else if (RxState == 1)
            {
                /* 等帧头第2字节 0x55 */
                if (Rxdata == 0x55)
                {
                    rx_buffer[1] = Rxdata;
                    pRxPacket = 2;
                    RxState = 2;
                }
                else
                {
                    RxState = 0;
                    pRxPacket = 0;
                }
            }
            else if (RxState == 2)
            {
                /* 收剩余字节: dx_h, dx_l, dy_h, dy_l, 0x0D */
                if (pRxPacket < 7)
                {
                    rx_buffer[pRxPacket] = Rxdata;
                    pRxPacket++;

                    if (pRxPacket >= 7)
                    {
                        /* 帧收完, 检查帧尾 */
                        if (rx_buffer[6] == 0x0D)
                        {
                            int16_t dx = (int16_t)(((uint16_t)rx_buffer[2] << 8) | rx_buffer[3]);
                            int16_t dy = (int16_t)(((uint16_t)rx_buffer[4] << 8) | rx_buffer[5]);

                            if (dx == VISION_ARRIVED_VAL && dy == VISION_ARRIVED_VAL)
                            {
                                /* 到位了 */
                                g_visual_stop_flag = 1;
                            }
                            else
                            {
                                /* 正常偏差帧 */
                                g_visual_dx = dx;
                                g_visual_dy = dy;
                            }
                        }
                        /* 无论帧尾对不对, 重新开始 */
                        RxState = 0;
                        pRxPacket = 0;
                    }
                }
                else
                {
                    RxState = 0;
                    pRxPacket = 0;
                }
            }
        }
    }
}

/* ========== 发给视觉的封装 ========== */
void K230_SendTaskCmd(uint8_t cmd)
{
   static uint8_t buf[3];
    buf[0] = 0xFF;
    buf[1] = cmd;
    buf[2] = 0xFE;
    K230_Dma_SendHex((uint8_t*)buf, 3);
}
void K230_SendQR(const char *qr)
{
   static char buf[16];
    uint8_t len = sprintf(buf, "$%s#", qr);
    K230_Dma_SendHex((uint8_t*)buf, len);
}



void Grap1_Wei_Tiao_Function(float g_target_yaw)
{
    uint8_t addr_map[4] = {0x05, 0x08, 0x06, 0x09};
    uint8_t i;

    /* PID 内部状态 (static, 跨调用保持) */
    static float vx_integrate  = 0.0f;
    static float vx_last_error = 0.0f;
    static float vy_integrate  = 0.0f;
    static float vy_last_error = 0.0f;
    static float wt_angle_integrate  = 0.0f;   /* Yaw PID 积分 (微调专用) */
    static float wt_angle_last_error = 0.0f;    /* Yaw PID 上次误差 (微调专用) */

    float s_wt_vx = 0.0f;
    float s_wt_vy = 0.0f;
    float s_wt_w  = 0.0f;

    /* ---- 1. 到位: 停车 + 复位所有PID ---- */
    if (g_visual_stop_flag) {
        vx_integrate = 0;  vx_last_error = 0;
        vy_integrate = 0;  vy_last_error = 0;
        wt_angle_integrate = 0;  wt_angle_last_error = 0;
    }
    else {
        /* ---- 2. PID 计算 vx (误差 = dx) ---- */
        /* dx>0偏右 → kp负 → vx<0 → 右移 */
        s_wt_vx = PID_vision1((float)(0-g_visual_dx), &vx_integrate, &vx_last_error,
                            1.6f, 0.0f, 1);

        /* ---- 3. PID 计算 vy (误差 = dy) ---- */
        /* dy>0偏下 → kp正 → vy>0 → 后移 */
        s_wt_vy = -PID_vision1((float)(0-g_visual_dy), &vy_integrate, &vy_last_error,
                             1.6f, 0.0f,1);

        /* ---- 4. Yaw 锁死 (和底盘同一套PID_angle, 微调专用状态变量) ---- */
        float error_angle = calculate_shortest_path_error(g_target_yaw - Current_World_Yaw);
        s_wt_w = PID_angle(error_angle, &wt_angle_integrate, &wt_angle_last_error,
                           1.2f, 0.0f, 0.8f);
    }

    /* ---- 5. 麦轮逆运动学 ---- */
    motor_v[0] = s_wt_vx - s_wt_vy - s_wt_w * CHASSIS_RADIUS;
    motor_v[1] = s_wt_vx + s_wt_vy + s_wt_w * CHASSIS_RADIUS;
    motor_v[2] = s_wt_vx + s_wt_vy - s_wt_w * CHASSIS_RADIUS;
    motor_v[3] = s_wt_vx - s_wt_vy + s_wt_w * CHASSIS_RADIUS;
    Delay_ms(1);
    /* ---- 6. 发送电机指令 ---- */
    for (i = 0; i < 4; i++) {
        if (motor_v[i] >= 0) {
            ZDT_Speed_Control(addr_map[i], straight, (uint16_t)(motor_v[i] * SPEED_RATIO));
        } else {
            ZDT_Speed_Control(addr_map[i], behind, (uint16_t)(-motor_v[i] * SPEED_RATIO));
        }
        Delay_ms(1);
    }
    ZDT_DJ_TongBu_Control();
		Delay_ms(2);
}

void Grap2_Wei_Tiao_Function(float g_target_yaw)
{
    uint8_t addr_map[4] = {0x05, 0x08, 0x06, 0x09};
    uint8_t i;

    /* PID 内部状态 (static, 跨调用保持) */
    static float vx_integrate  = 0.0f;
    static float vx_last_error = 0.0f;
    static float vy_integrate  = 0.0f;
    static float vy_last_error = 0.0f;
    static float wt_angle_integrate  = 0.0f;   /* Yaw PID 积分 (微调专用) */
    static float wt_angle_last_error = 0.0f;    /* Yaw PID 上次误差 (微调专用) */

    float s_wt_vx = 0.0f;
    float s_wt_vy = 0.0f;
    float s_wt_w  = 0.0f;

    /* ---- 1. 到位: 停车 + 复位所有PID ---- */
    if (g_visual_stop_flag) {
        vx_integrate = 0;  vx_last_error = 0;
        vy_integrate = 0;  vy_last_error = 0;
        wt_angle_integrate = 0;  wt_angle_last_error = 0;
    }
    else {
							
									/* ---- 1. 独立判断并计算 X 轴 ---- */
					if (abs(g_visual_dx) > 4.0f && abs(g_visual_dx) < 15.0f) {
							/* 情况A：X轴接近目标，减速模式 (结果除以 16.2f) */
							s_wt_vx = PID_vision2((float)(0 - g_visual_dx), &vx_integrate, &vx_last_error,
																		1.6f, 0.0f, 1) / 30.0f;
					} 
					else if (abs(g_visual_dx) <= 4.0f) {
							/* 情况C：X轴偏差极小，正常全速模式 */
							s_wt_vx = PID_vision2((float)(0 - g_visual_dx), &vx_integrate, &vx_last_error,
																		1.6f, 0.0f, 1)/27.0f;
					} 
					else {
							/* 情况B：X轴远离目标 (>=15)，正常全速模式 */
							s_wt_vx = PID_vision2((float)(0 - g_visual_dx), &vx_integrate, &vx_last_error,
																		1.6f, 0.0f, 1);
					}

					/* ---- 2. 独立判断并计算 Y 轴 ---- */
					if (abs(g_visual_dy) > 4.0f && abs(g_visual_dy) < 15.0f) {
							/* 情况A：Y轴接近目标，减速模式 (结果除以 16.2f) */
							s_wt_vy = -PID_vision2((float)(0 - g_visual_dy), &vy_integrate, &vy_last_error,
																		 1.6f, 0.0f, 1) / 30.0f;
					} 
					else if (abs(g_visual_dy) <= 4.0f) {
							/* 情况C：Y轴偏差极小，正常全速模式 */
							s_wt_vy = -PID_vision2((float)(0 - g_visual_dy), &vy_integrate, &vy_last_error,
																		 1.6f, 0.0f, 1)/27.0f;
					} 
					else {
							/* 情况B：Y轴远离目标 (>=15)，正常全速模式 */
							s_wt_vy = -PID_vision2((float)(0 - g_visual_dy), &vy_integrate, &vy_last_error,
																		 1.6f, 0.0f, 1);
					}


        /* ---- 4. Yaw 锁死 (保持不变) ---- */
        float error_angle = calculate_shortest_path_error(g_target_yaw - Current_World_Yaw);
        s_wt_w = PID_angle(error_angle, &wt_angle_integrate, &wt_angle_last_error,
                           2.0f, 0.0f, 1.6f);
    

    }

    /* ---- 5. 麦轮逆运动学 ---- */
    motor_v[0] = s_wt_vx - s_wt_vy - s_wt_w * CHASSIS_RADIUS;
    motor_v[1] = s_wt_vx + s_wt_vy + s_wt_w * CHASSIS_RADIUS;
    motor_v[2] = s_wt_vx + s_wt_vy - s_wt_w * CHASSIS_RADIUS;
    motor_v[3] = s_wt_vx - s_wt_vy + s_wt_w * CHASSIS_RADIUS;
    Delay_ms(1);
    /* ---- 6. 发送电机指令 ---- */
    for (i = 0; i < 4; i++) {
        if (motor_v[i] >= 0) {
            ZDT_Speed_Control(addr_map[i], straight, (uint16_t)(motor_v[i] * SPEED_RATIO));
        } else {
            ZDT_Speed_Control(addr_map[i], behind, (uint16_t)(-motor_v[i] * SPEED_RATIO));
        }
        Delay_ms(1);
    }
    ZDT_DJ_TongBu_Control();
		Delay_ms(2);
}