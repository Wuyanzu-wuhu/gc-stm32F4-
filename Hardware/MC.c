#include "stm32f4xx.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "MC.h"
#include "math.h"
#include "HWT101.h"
#include "PID.h"
#include "Delay.h"
#include "control.h"

/* ========== 引脚映射 (F4) ==========
   USART3_TX -> PB10  (AF7)
   USART3_RX -> PB11  (AF7)
   RX DMA    -> DMA1_Stream1_Channel4
   TX DMA    -> DMA1_Stream3_Channel4
   ================================== */

// ===== 私有变量 =====
static uint32_t sc_t_start = 0;
static uint32_t t_acc_val = 0;
static uint32_t t_cru_val = 0;
static uint32_t t_dec_val = 0;
static float sc_vx1 = 0, sc_vy1 = 0;
static float sc_vx2 = 0, sc_vy2 = 0;
float g_target_yaw = 0;
static uint8_t turn_stable_cnt = 0;

uint8_t g_sc_state = SC_IDLE;
float motor_v[4] = {0};
float  pid_input_error;
float vx_body;
float vy_body; 


float angle_straight_integrate=0;
float angle_straight_last_error=0;

// ====== DMA 接收缓冲区 ======
#define DMA_RX_BUF_SIZE 128
uint8_t DMA1_RxBuf[DMA_RX_BUF_SIZE]; // USART3 专用 DMA 接收缓存

// ====== 全局变量 ======
uint8_t DJ_DaoWei_Data = 0;
uint8_t DJ_HuiLing_Data = 0;
char DuoJi_MaiChong_Data_Buffer[16] = {0};
uint16_t DuoJi_MaiChong_Data = 0;


void MC_DMA_Serial_Init(void)
{
    /* ---- 1. 开时钟 (F4: GPIO/DMA在AHB1, USART3在APB1) ---- */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* ---- 2. GPIO -- PB10(TX) / PB11(RX) ---- */
    GPIO_InitTypeDef GPIO_InitStructure;
    // PB10 - USART3_TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);

    // PB11 - USART3_RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    /* ---- 3. USART3 参数 ---- */
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART3, &USART_InitStructure);

    /* ---- 4. DMA 接收 (DMA1_Stream1_Channel4) ---- */
    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA1_Stream1);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DMA1_RxBuf;
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
    DMA_Init(DMA1_Stream1, &DMA_InitStructure);

    /* ---- 5. DMA 发送 (DMA1_Stream3_Channel4) ---- */
    DMA_DeInit(DMA1_Stream3);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
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
    DMA_Init(DMA1_Stream3, &DMA_InitStructure);

    /* ---- 6. 中断 ---- */
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* ---- 7. 使能 ---- */
    USART_Cmd(USART3, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Stream1, ENABLE); // 开启DMA接收
}

void Serial_SendHex_DMA(uint8_t *Data, uint16_t Length)
{
    // 步骤1：如果 DMA 正在运行，等待上一次传输完成
    if((DMA1_Stream3->CR & DMA_SxCR_EN) != RESET)
    {
        while(DMA_GetFlagStatus(DMA1_Stream3, DMA_FLAG_TCIF3) == RESET);
    }

    // 步骤2：软件关闭 DMA 通道
    DMA_Cmd(DMA1_Stream3, DISABLE);

    // 步骤3：等待硬件真正关闭
    while((DMA1_Stream3->CR & DMA_SxCR_EN) != RESET);

    // 步骤4：清除 DMA 所有标志位
    DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_FEIF3);

    // 步骤5：重新配置内存地址和传输数量
    DMA1_Stream3->M0AR = (uint32_t)Data;
    DMA1_Stream3->NDTR = Length;

    // 步骤6：启动 DMA
    DMA_Cmd(DMA1_Stream3, ENABLE);
}

/***************************************************************
 *             USART3 空闲中断处理 (DMA接收解析)               *
 ***************************************************************/
void USART3_IRQHandler(void)
{
    static uint8_t RxState = 0; 
    static uint8_t GongNeng_ma = 0;
    static char LingShi_Buffer[16] = {0};  
    static uint8_t pRxPacket = 0;

    // ORE错误处理
    if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET) {
        volatile uint8_t d = USART3->DR;
        d = USART3->SR;
        (void)d;
    }

    // IDLE空闲中断处理
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        volatile uint32_t t = USART3->SR;
        t = USART3->DR;
        (void)t;
        USART_ReceiveData(USART3);
        uint16_t rx_len = DMA_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1);

    
			  DMA_Cmd(DMA1_Stream1, DISABLE);
			    while (DMA1_Stream1->CR & DMA_SxCR_EN);   // 必须等待！
			   DMA1->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1          //这个最关键了
                    | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;
        DMA_SetCurrDataCounter(DMA1_Stream1, DMA_RX_BUF_SIZE);
        DMA_Cmd(DMA1_Stream1, ENABLE);

        for (uint16_t i = 0; i < rx_len; i++)
        {
            uint8_t RxData = DMA1_RxBuf[i];

            switch (RxState)
            {
                case 0: 
                    if (RxData == '#') {
                        RxState = 4;
                    }
                    else if (RxData == 0x05 || RxData == 0x06 || RxData == 0x08 || RxData == 0x09) {
                        RxState = 1;
                    }
                    break;

                case 1: 
                    if (RxData == 0x3A || RxData == 0x3B) {
                        GongNeng_ma = RxData;
                        RxState = 2;
                    } else {
                        RxState = 0; 
                    }
                    break;

                case 2: 
                    if(GongNeng_ma == 0x3A) {
                        DJ_DaoWei_Data = RxData;
                        RxState = 3;
                    }
                    if(GongNeng_ma == 0x3B) {
                        DJ_HuiLing_Data = RxData;
                        RxState = 3;
                    }
                    break;

                case 3: 
                    if (RxData == 0x6B) {
                        GongNeng_ma = 0;
                        RxState = 0;   
                    }
                    break;

                case 4: 
                    if (RxData == '!') { 
                        LingShi_Buffer[pRxPacket] = '\0';
                        memcpy(&DuoJi_MaiChong_Data_Buffer[0], &LingShi_Buffer[4], 4);
                        DuoJi_MaiChong_Data_Buffer[4] = '\0';
                        DuoJi_MaiChong_Data = (uint16_t)atoi(DuoJi_MaiChong_Data_Buffer);
                        pRxPacket = 0;
                        RxState = 0; 
                    } else {
                        LingShi_Buffer[pRxPacket] = RxData;
                        pRxPacket++;
                    }
                    break;
            }
        }
    }
}

/***************************************************************
 *             以下是业务逻辑发送函数(改为DMA发送)              *
 ***************************************************************/

// 1. 位置控制
void ZDT_Position_Control(uint8_t Addr, DJ_Direction_Type dir, uint16_t speed, uint32_t clk)
{  
    static uint8_t cmd[16] = {0};
    cmd[0] = Addr;
    cmd[1] = 0xFD;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(speed >> 8);
    cmd[4] = (uint8_t)(speed >> 0);
    cmd[5] = 0x00;
    cmd[6] = (uint8_t)(clk >> 24);
    cmd[7] = (uint8_t)(clk >> 16);
    cmd[8] = (uint8_t)(clk >> 8);
    cmd[9] = (uint8_t)(clk >> 0);
    cmd[10] = 0x00;
    cmd[11] = 0x00;
    cmd[12] = 0x6B;

    Serial_SendHex_DMA(cmd, 13);
}

void ZDT_JueDui_Position_Control(uint8_t Addr, DJ_Direction_Type dir, uint16_t speed, uint8_t acc, uint32_t clk)  //0x02加速度是0就行，0x03加速度下落的时候加加速度
{
    static uint8_t cmd[16] = {0};
    cmd[0] = Addr;
    cmd[1] = 0xFD;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(speed >> 8);
    cmd[4] = (uint8_t)(speed >> 0);
    cmd[5] = acc;
    cmd[6] = (uint8_t)(clk >> 24);
    cmd[7] = (uint8_t)(clk >> 16);
    cmd[8] = (uint8_t)(clk >> 8);
    cmd[9] = (uint8_t)(clk >> 0);
    cmd[10] = 0x01;
    cmd[11] = 0x00;         //不同步控制
    cmd[12] = 0x6B;

    Serial_SendHex_DMA(cmd, 13);
}

// 2. 速度控制
void ZDT_Speed_Control(uint8_t Addr, DJ_Direction_Type dir, uint16_t speed)
{  
    static uint8_t cmd[8] = {0};
    cmd[0] = Addr;
    cmd[1] = 0xF6;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(speed >> 8);
    cmd[4] = (uint8_t)(speed >> 0);
    cmd[5] = 0x00;
    cmd[6] = 0x01;         //01是同步控制
    cmd[7] = 0x6B;

    Serial_SendHex_DMA(cmd, 8);
}

// 3. 电机同步
void ZDT_DJ_TongBu_Control(void)
{  
    static uint8_t cmd[8] = {0};
    cmd[0] = 0x00;
    cmd[1] = 0xFF;
    cmd[2] = 0x66;
    cmd[3] = 0x6B;

    Serial_SendHex_DMA(cmd, 4);
}

// 4. 查询到位
void ZDT_Send_DJ_DaoWei_Control(uint8_t Addr)
{
    static uint8_t cmd[8] = {0};
    cmd[0] = Addr;   
    cmd[1] = 0x3A;   
    cmd[2] = 0x6B;  

    Serial_SendHex_DMA(cmd, 3);
}

// 5. 查询回零
void ZDT_Send_DJ_HuiLing_Control(uint8_t Addr)
{
    static uint8_t cmd[8] = {0};
    cmd[0] = Addr;   
    cmd[1] = 0x3B;   
    cmd[2] = 0x6B;  

    Serial_SendHex_DMA(cmd, 3);
}



// 以下获取标志位的函数保持不变
uint8_t ZDT_Get_DJ_DaoWei_Flag(void)
{ 
    if(DJ_DaoWei_Data & 0x02) return 1;
    else return 0;
}

uint8_t ZDT_Get_DJ_DaoWei_Sum_Flag(void)
{  
    static uint8_t number = 0;  
    if(ZDT_Get_DJ_DaoWei_Flag() == 1) {
        number++;
    }
    if(number >= 5) {
        number = 0;
        return 1;
    }
    return 0;
}

uint8_t ZDT_Get_DJ_HuiLing_Flag(void)    
{ 
    if((DJ_HuiLing_Data & 0x0C) == 0x00) return 1;
    else if((DJ_HuiLing_Data & 0x0C) == 0x04) return 0;
    return 0;
}


void Chassis_Core(PID_Mode_Type mode, float target_yaw, 
                  float vx1, float vy1, float vx2, float vy2,
                  uint32_t t_acc, uint32_t t_cru, uint32_t t_dec, 
                  uint8_t start_flag) 
{

    uint8_t addr_map[4] = {0x05, 0x08, 0x06, 0x09};


    // ---------- A. 启动初始化 ----------
    if (start_flag == 1) {
        PID_Mode = mode;
        g_target_yaw  = target_yaw;

       uint8_t need_reset_pid = 0;
    if (mode == Angle) {
        need_reset_pid = 1;
    } else if (vx1 == 0.0f && vy1 == 0.0f) {
        need_reset_pid = 1;
    }

    if (need_reset_pid) {
        angle_straight_integrate = 0.0f;
        angle_straight_last_error = 0.0f;
    }

        turn_stable_cnt = 0;

        if (PID_Mode == Straight_and_Angle) {
            sc_vx1 = vx1; sc_vy1 = vy1;
            sc_vx2 = vx2; sc_vy2 = vy2;
            t_acc_val = t_acc;
            t_cru_val = t_cru;
            t_dec_val = t_dec;
            sc_t_start = System_Tick_10ms;
            g_sc_state = SC_ACCEL;
        } 
        else {
            sc_t_start = System_Tick_10ms;
            g_sc_state = SC_TURNING;
        }
    }

    // ---------- B. 状态机计算 ----------
    switch (g_sc_state) {

        /* ===================== 直走分支 ===================== */
        case SC_ACCEL: {
            uint32_t elapsed = System_Tick_10ms - sc_t_start;
            float vx_world_cur = 0, vy_world_cur = 0;

            if (elapsed >= t_acc_val) {
                vx_world_cur = sc_vx2; vy_world_cur = sc_vy2;
                g_sc_state = SC_CRUISE;
                sc_t_start = System_Tick_10ms;
            } else {
                float ratio = 0.5f - 0.5f * cosf(PI / t_acc_val * elapsed);
                vx_world_cur = sc_vx1 + (sc_vx2 - sc_vx1) * ratio;
                vy_world_cur = sc_vy1 + (sc_vy2 - sc_vy1) * ratio;
            }

            float theta = Current_World_Yaw * PI / 180.0f;
             vx_body =  vx_world_cur * cosf(theta) + vy_world_cur * sinf(theta);
            vy_body = -vx_world_cur * sinf(theta) + vy_world_cur * cosf(theta);

            float error_angle = calculate_shortest_path_error(g_target_yaw - Current_World_Yaw);
            pid_input_error = error_angle;
           float w = PID_angle(pid_input_error, &angle_straight_integrate,
                          &angle_straight_last_error, 1.6, 0.0f, 1);

            motor_v[0] = vx_body - vy_body - w * CHASSIS_RADIUS;
            motor_v[1] = vx_body + vy_body + w * CHASSIS_RADIUS;
            motor_v[2] = vx_body + vy_body - w * CHASSIS_RADIUS;
            motor_v[3] = vx_body - vy_body + w * CHASSIS_RADIUS;
            break;
        }

        case SC_CRUISE: {
            uint32_t elapsed = System_Tick_10ms - sc_t_start;
            float vx_world_cur = sc_vx2, vy_world_cur = sc_vy2;

            if (elapsed >= t_cru_val) {
                if (t_dec_val == 0) {
            g_sc_state = SC_IDLE;
        } else {
            g_sc_state = SC_DECEL;
            sc_t_start = System_Tick_10ms;
                     sc_vx1=0,sc_vy1=0;
        }
            }

            float theta = Current_World_Yaw * PI / 180.0f;
            vx_body =  vx_world_cur * cosf(theta) + vy_world_cur * sinf(theta);
            vy_body = -vx_world_cur * sinf(theta) + vy_world_cur * cosf(theta);

           float error_angle = calculate_shortest_path_error(g_target_yaw - Current_World_Yaw);
            pid_input_error = error_angle;
          float  w = PID_angle(pid_input_error, &angle_straight_integrate,
                          &angle_straight_last_error, 1.6, 0.0f, 1);

            motor_v[0] = vx_body - vy_body - w * CHASSIS_RADIUS;
            motor_v[1] = vx_body + vy_body + w * CHASSIS_RADIUS;
            motor_v[2] = vx_body + vy_body - w * CHASSIS_RADIUS;
            motor_v[3] = vx_body - vy_body + w * CHASSIS_RADIUS;
            break;
        }

        case SC_DECEL: {
            uint32_t elapsed = System_Tick_10ms - sc_t_start;
            float vx_world_cur = 0, vy_world_cur = 0;

           if (elapsed >= t_dec_val) {

            g_sc_state = SC_GUIWEI;
            sc_t_start = System_Tick_10ms;
            angle_straight_integrate = 0.0f;
            angle_straight_last_error = 0.0f;

       } 
                     else {
                float ratio = 0.5f - 0.5f * cosf(PI / t_dec_val * elapsed);
                vx_world_cur = sc_vx2 + (sc_vx1 - sc_vx2) * ratio;
                vy_world_cur = sc_vy2 + (sc_vy1 - sc_vy2) * ratio;

                float theta = Current_World_Yaw * PI / 180.0f;
                vx_body =  vx_world_cur * cosf(theta) + vy_world_cur * sinf(theta);
               vy_body = -vx_world_cur * sinf(theta) + vy_world_cur * cosf(theta);

               float error_angle = calculate_shortest_path_error(g_target_yaw - Current_World_Yaw);
            pid_input_error = error_angle;
              float  w = PID_angle(pid_input_error, &angle_straight_integrate,
                              &angle_straight_last_error,1.6, 0.0f, 1);

                motor_v[0] = vx_body - vy_body - w * CHASSIS_RADIUS;
                motor_v[1] = vx_body + vy_body + w * CHASSIS_RADIUS;
                motor_v[2] = vx_body + vy_body - w * CHASSIS_RADIUS;
                motor_v[3] = vx_body - vy_body + w * CHASSIS_RADIUS;
            }
            break;
        }
        
        case SC_GUIWEI: {
            uint32_t elapsed = System_Tick_10ms - sc_t_start;
            
            if (elapsed >= 100) {
                g_sc_state = SC_IDLE;
                motor_v[0] = motor_v[1] = motor_v[2] = motor_v[3] = 0;
            } else {
               float error_angle = calculate_shortest_path_error(g_target_yaw - Current_World_Yaw);
              pid_input_error = error_angle;
              float  w = PID_angle(pid_input_error, &angle_straight_integrate,
                              &angle_straight_last_error, 1.6, 0.0f, 1);
                
                motor_v[0] = -w * CHASSIS_RADIUS;
                motor_v[1] =  w * CHASSIS_RADIUS;
                motor_v[2] = -w * CHASSIS_RADIUS;
                motor_v[3] =  w * CHASSIS_RADIUS;
            }
            break;
        }
        
        /* ===================== 纯旋转分支 ===================== */
        case SC_TURNING: {
           float error_angle = calculate_shortest_path_error(g_target_yaw - Current_World_Yaw);
            pid_input_error = error_angle;
          float  w = PID_angle(pid_input_error, &angle_straight_integrate,
                          &angle_straight_last_error, 1.6, 0.0f, 1);
            
            motor_v[0] = -w * CHASSIS_RADIUS;
            motor_v[1] =  w * CHASSIS_RADIUS;
            motor_v[2] = -w * CHASSIS_RADIUS;
            motor_v[3] =  w * CHASSIS_RADIUS;
            
            if (fabsf(error_angle) < 1.0f) {
                if (++turn_stable_cnt >= 5) {
                    g_sc_state = SC_IDLE;
                    motor_v[0] = motor_v[1] = motor_v[2] = motor_v[3] = 0;
                }
            } else {
                turn_stable_cnt = 0;
            }
            break;
        }
        
        case SC_IDLE:
        default:
            motor_v[0] = motor_v[1] = motor_v[2] = motor_v[3] = 0;
            break;
    }
    
    // ---------- C. 统一发送电机指令 ----------
        uint8_t i;
    for (i = 0; i < 4; i++) {
        if (motor_v[i] >= 0) {
            ZDT_Speed_Control(addr_map[i], straight, (uint16_t)(motor_v[i] * SPEED_RATIO));
        } else {
            ZDT_Speed_Control(addr_map[i], behind, (uint16_t)(-motor_v[i] * SPEED_RATIO));
        }
        Delay_ms(1);
    }
    ZDT_DJ_TongBu_Control();
}
