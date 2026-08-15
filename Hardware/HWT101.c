#include "stm32f4xx.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ========== 引脚映射 (F4) ==========
   UART4_TX -> PC10  (AF8)
   UART4_RX -> PC11  (AF8)
   RX DMA   -> DMA1_Stream2_Channel4
   TX       -> 轮询 (与F1一致)
   ================================== */

#define DMA_RX_BUF_SIZE 128  // 缓冲区大小
uint8_t DMA_RxBuf[DMA_RX_BUF_SIZE];

// 全局变量
uint8_t  rx_buffer[20];  // 11字节标准帧
volatile float  current_yaw = 0; // 偏航角（度）
volatile float  accel_z = 0;     // Z轴加速度（mg）

volatile float Start_Yaw = 0;      // 上电时的角度（世界0度基准）
volatile float LingShi_Current_World_Yaw = 0;      //缓存作用
volatile float Current_World_Yaw = 0;         // 世界坐标系下的实时角度
volatile uint8_t Start_Yaw_Flag = 0;    // 一次性标志位--上电角度

float calculate_shortest_path_error(float error) //转化成最佳路径角度
{
    if (error > 180.0f) {
        error = error-360.0f;
    } else if (error < -180.0f) {
        error = error+360.0f;
    }
    return error;
}

void Hwt101_Dma_Serial_Init(void)
{
    /* ---- 1. 开启时钟 (F4: GPIO在AHB1, UART4在APB1, DMA在AHB1) ---- */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    /* ---- 2. GPIO 初始化 (F4: 需要 OType, PuPd, PinAFConfig) ---- */
    GPIO_InitTypeDef GPIO_InitStructure;
    // PC10 - UART4_TX (AF8, 推挽复用)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);

    // PC11 - UART4_RX (AF8, 上拉)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

    /* ---- 3. UART4 参数配置 ---- */
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 230400;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(UART4, &USART_InitStructure);

    /* ---- 4. DMA 接收配置 (DMA1_Stream2_Channel4) ---- */
    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA1_Stream2);  // 先复位
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DMA_RxBuf;
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
    DMA_Init(DMA1_Stream2, &DMA_InitStructure);

    /* ---- 5. 中断配置 (UART4 IDLE中断) ---- */
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* ---- 6. 使能 ---- */
    USART_Cmd(UART4, ENABLE);
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(DMA1_Stream2, ENABLE);
}

/**
 * UART4 空闲中断处理 (DMA接收 + 帧解析)
 */
void UART4_IRQHandler(void) {
    static uint8_t RxState = 0;
    static uint8_t DataType = 0;
    static int pRxPacket = 0;

    // ORE 溢出错误处理
    if(USART_GetFlagStatus(UART4, USART_FLAG_ORE) != RESET) {
        volatile uint8_t d = UART4->DR;
        d = UART4->SR;
        (void)d;
    }

    // IDLE 空闲中断
    if (USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
    {
        volatile uint32_t t = UART4->SR;  // 清 IDLE
        t = UART4->DR;
        (void)t;
        USART_ReceiveData(UART4);

        // 计算接收长度
        uint16_t rx_len = DMA_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream2);

        DMA_Cmd(DMA1_Stream2, DISABLE);    
			    while (DMA1_Stream2->CR & DMA_SxCR_EN);   // 必须等待！
			   DMA1->LIFCR = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2          //这个最关键了
                    | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2;
        DMA_SetCurrDataCounter(DMA1_Stream2, DMA_RX_BUF_SIZE);
        DMA_Cmd(DMA1_Stream2, ENABLE);

        for (uint16_t i = 0; i < rx_len; i++)
        {
            uint8_t Rxdata = DMA_RxBuf[i];

            if(RxState == 0)
            {
                if (Rxdata == 0x55)
                {
                    rx_buffer[0] = Rxdata;
                    pRxPacket++;
                    RxState = 1;
                }
            }
            else if(RxState == 1)
            {
                if (Rxdata == 0x53)
                {
                    DataType = Rxdata;
                    rx_buffer[1] = Rxdata;
                    pRxPacket++;
                    RxState = 2;
                }
            }
            else if(RxState == 2)
            {
                if(pRxPacket < 11)
                {
                    rx_buffer[pRxPacket] = Rxdata;
                    pRxPacket++;
                }

                if(pRxPacket >= 11)
                {
                    // 校验和
                    uint8_t sum = 0;
                    for(int j = 0; j < 10; j++) {
                        sum += rx_buffer[j];
                    }

                    if(sum == rx_buffer[10])
                    {
                        if (DataType == 0x53) {
                            int16_t yaw_raw = (int16_t)((rx_buffer[7] << 8) | rx_buffer[6]);
                            current_yaw = yaw_raw * (180.0f / 32768.0f);
                            if (Start_Yaw_Flag == 0)
                            {
                                Start_Yaw = current_yaw;
                                Start_Yaw_Flag = 1;
                            }
                            LingShi_Current_World_Yaw = current_yaw - Start_Yaw;
                            Current_World_Yaw = calculate_shortest_path_error(LingShi_Current_World_Yaw);
                        }
                        else if (DataType == 0x51) {
                            int16_t az_raw = (int16_t)((rx_buffer[7] << 8) | rx_buffer[6]);
                            accel_z = (az_raw * 16.0f * 1000.0f) / 32768.0f;
                        }
                    }
                    pRxPacket = 0;
                    RxState = 0;
                }
            }
        }
    }
}


void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(UART4, Byte);
    while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
}


void Serial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        Serial_SendByte(String[i]);
    }
}

void Serial_SendHex(uint8_t *Data, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++)
    {
        Serial_SendByte(Data[i]);
    }
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y --)
    {
        Result *= X;
    }
    return Result;
}


void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

int fputc(int ch, FILE *f)
{
    Serial_SendByte(ch);
    return ch;
}

void Serial_Printf(char *format, ...)
{
    char String[256];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    Serial_SendString(String);
}
