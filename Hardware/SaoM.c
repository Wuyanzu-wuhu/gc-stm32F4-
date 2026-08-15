#include "SaoM.h"
#include "misc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ========== 引脚映射 (F4) ==========
   USART6_TX -> PC6   (AF8)
   USART6_RX -> PC7   (AF8)
   RX DMA    -> DMA2_Stream1_Channel5
   TX        -> 轮询发送 (扫码应答简单)
   ================================== */

/*==================== 接收缓存及全局变量 ====================*/

#define DMA_RX_BUF_SIZE 64
uint8_t DMA_SaoM_RxBuf[DMA_RX_BUF_SIZE];

char usart3_rx_buffer[RX_BUFFER_SIZE];
uint8_t usart3_rx_index = 0;
volatile uint8_t SaoM_complete_Flag = 0;

int received_number = 0;
int task1, task2, task3;
int task4, task5, task6;
int text1 = 0, text2 = 0;

/*==================== ★ 改动1：新增屏蔽计数器 ====================*/
static volatile uint8_t gm65_ignore_rx_count = 0u;

/*==================== USART6 DMA 初始化 (PC6/PC7) ====================*/

void SaoM_Dma_Serial_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /*--- 1. 开时钟 (F4: GPIO/DMA在AHB1, USART6在APB2) ---*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    /*--- 2. GPIO 配置 (F4: 需要 OType, PuPd, PinAFConfig) ---*/
    // PC6 - USART6_TX (AF8)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);

    // PC7 - USART6_RX (AF8, 浮空输入，扫码枪一般推挽输出)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

    /*--- 3. USART6 参数 9600-8-N-1 ---*/
    USART_InitStruct.USART_BaudRate = 9600;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART6, &USART_InitStruct);

    /*--- 4. DMA 接收 (DMA2_Stream1_Channel5) ---*/
    DMA_InitTypeDef DMA_InitStruct;
    DMA_DeInit(DMA2_Stream1);
    DMA_InitStruct.DMA_Channel = DMA_Channel_5;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)DMA_SaoM_RxBuf;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStruct.DMA_BufferSize = DMA_RX_BUF_SIZE;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream1, &DMA_InitStruct);

    /*--- 5. 使能 IDLE 空闲中断 ---*/
    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);

    /*--- 6. NVIC 配置 ---*/
    NVIC_InitStruct.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    /*--- 7. 使能 ---*/
    USART_Cmd(USART6, ENABLE);
    USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(DMA2_Stream1, ENABLE);

    /*--- 8. 初始化接收缓存 ---*/
    memset(usart3_rx_buffer, 0, RX_BUFFER_SIZE);
    usart3_rx_index = 0;
    SaoM_complete_Flag = 0;
    /*==================== ★ 改动2：初始化屏蔽计数器 ====================*/
    gm65_ignore_rx_count = 0;
}

/*==================== USART6 DMA+IDLE 中断服务函数 ====================*/

void USART6_IRQHandler(void)
{
    // ORE 溢出错误处理
    if(USART_GetFlagStatus(USART6, USART_FLAG_ORE) != RESET) {
        volatile uint8_t d = USART6->DR;
        d = USART6->SR;
        (void)d;
    }

    // IDLE 空闲中断处理
    if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
    {
        volatile uint32_t t = USART6->SR;
        t = USART6->DR;
        (void)t;
        USART_ReceiveData(USART6);

        uint16_t rx_len = DMA_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA2_Stream1);

        // 重启DMA
        DMA_Cmd(DMA2_Stream1, DISABLE);      
        while (DMA2_Stream1->CR & DMA_SxCR_EN);   // 必须等待！
        DMA2->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1
                    | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;
        DMA_SetCurrDataCounter(DMA2_Stream1, DMA_RX_BUF_SIZE);
        DMA_Cmd(DMA2_Stream1, ENABLE);

        // 逐字节解析 (保持原F1的状态机逻辑)
        for (uint16_t i = 0; i < rx_len; i++)
        {
            char received_char = (char)DMA_SaoM_RxBuf[i];

            /*==================== ★ 改动3：屏蔽应答字节 ====================*/
            // 先处理屏蔽计数
            if(gm65_ignore_rx_count > 0u)
            {
                gm65_ignore_rx_count--;
                continue;
            }

            // 如果已完成接收，忽略后续字符
            if(SaoM_complete_Flag)
                continue;

            /*--- 严格状态机：只接收 "123+456" 这种 7 字节定长帧 ---*/
            if(usart3_rx_index < 3)                 // 索引 0/1/2：必须是数字
            {
                if(received_char >= '0' && received_char <= '9')
                {
                    usart3_rx_buffer[usart3_rx_index++] = received_char;
                }
                else
                {
                    usart3_rx_index = 0;
                    memset(usart3_rx_buffer, 0, RX_BUFFER_SIZE);
                }
            }
            else if(usart3_rx_index == 3)           // 索引 3：必须是 '+'
            {
                if(received_char == '+')
                {
                    usart3_rx_buffer[usart3_rx_index++] = received_char;
                }
                else
                {
                    usart3_rx_index = 0;
                    memset(usart3_rx_buffer, 0, RX_BUFFER_SIZE);
                }
            }
            else if(usart3_rx_index < 7)            // 索引 4/5/6：必须是数字
            {
                if(received_char >= '0' && received_char <= '9')
                {
                    usart3_rx_buffer[usart3_rx_index++] = received_char;

                    if(usart3_rx_index == 7)        // 收满 7 字节
                    {
                        usart3_rx_buffer[usart3_rx_index] = '\0';
                        SaoM_complete_Flag = 1;
                    }
                }
                else
                {
                    usart3_rx_index = 0;
                    memset(usart3_rx_buffer, 0, RX_BUFFER_SIZE);
                }
            }
            else
            {
                usart3_rx_index = 0;
                memset(usart3_rx_buffer, 0, RX_BUFFER_SIZE);
            }
        }
    }
}

/*==================== 解析函数 ====================*/

uint8_t ParseTwoNumbers(const char *str, int *num1, int *num2)
{
    const char *plus_pos = strchr(str, '+');
    if(plus_pos == NULL)
        return 1;

    int len_left  = plus_pos - str;
    int len_right = strlen(plus_pos + 1);

    if(len_left != 3 || len_right != 3)
        return 1;

    for(int i = 0; i < 3; i++)
    {
        if(str[i] < '0' || str[i] > '9')
            return 1;
        if(plus_pos[1 + i] < '0' || plus_pos[1 + i] > '9')
            return 1;
    }

    char temp[4];
    strncpy(temp, str, 3);      temp[3] = '\0'; *num1 = atoi(temp);
    strncpy(temp, plus_pos + 1, 3); temp[3] = '\0'; *num2 = atoi(temp);

    return 0;
}


/*==================== 发送接口 ====================*/

void USART6_SendByte(uint8_t data)
{
    while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
    USART_SendData(USART6, data);
    while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
}

void USART6_SendData(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    for(i = 0u; i < len; i++)
    {
        USART6_SendByte(data[i]);
    }
}
void Scan_Function(void)
{
    static const uint8_t triggerCmd[9] = {
        0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD
    };

    /*==================== ★ 改动4：发送前屏蔽应答 ====================*/
    usart3_rx_index = 0;
    memset(usart3_rx_buffer, 0, RX_BUFFER_SIZE);
    SaoM_complete_Flag = 0;
    gm65_ignore_rx_count = 7u;

    USART6_SendData(triggerCmd, 9);
}


void USART6_SendString(char *str)
{
    while(*str)
    {
        USART6_SendByte(*str++);
    }
}

/*==================== 简易查询接口（兼容旧代码） ====================*/

void Receive_QR(void)
{
    if(SaoM_complete_Flag)
    {
        received_number = usart3_rx_buffer[1];

        usart3_rx_index = 0;
        memset(usart3_rx_buffer, 0, RX_BUFFER_SIZE);
        SaoM_complete_Flag = 0;
    }
}


/*==================== 主循环调用：解析并提取各位 ====================*/

void ProcessReceivedExpression(void)
{
    if(SaoM_complete_Flag)
    {
        int i = 0;
        int a, b;

        if(ParseTwoNumbers(usart3_rx_buffer, &a, &b) == 0)
        {
            text1 = a;
            text2 = b;

            task1 = a / 100;
            task2 = (a / 10) % 10;
            task3 = a % 10;

            task4 = b / 100;
            task5 = (b / 10) % 10;
            task6 = b % 10;

            if(text1 != 0)
            {
                while(i < 10)
                {
                    // USART2_SendFormattedString(usart3_rx_buffer);
                    i++;
                }
            }
        }

        usart3_rx_index = 0;
        memset(usart3_rx_buffer, 0, RX_BUFFER_SIZE);
        SaoM_complete_Flag = 0;
    }
}