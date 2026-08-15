#include "stm32f4xx.h"

// 微秒延时（基于 TIM4 计数器，不占用 SysTick）
// F4: APB1=42MHz, TIM4 定时器时钟=84MHz (2x APB1 when prescaler≠1)
// Prescaler=84-1 → 1MHz (1us/tick), Period=10000-1 → 10ms 周期
void count_TIM4_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_InternalClockConfig(TIM4);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;      // F4: 84MHz/84=1MHz=1us
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
    // 不需要中断、不需要 NVIC
    TIM_Cmd(TIM4, ENABLE);   // 启动定时器即可
}

void Delay_us(uint32_t xus)
{
    uint32_t last = TIM4->CNT;
    uint32_t elapsed = 0;
    uint32_t now;
    while (elapsed < xus)
    {
        now = TIM4->CNT;
        if (now >= last)
            elapsed += (now - last);
        else
            elapsed += (10000 - last + now); // 计数器溢出复位后累加整个周期
        last = now;
    }
}

// 毫秒延时
void Delay_ms(uint32_t xms)
{
    while (xms--)
        Delay_us(1000);
}

// 秒延时
void Delay_s(uint32_t xs)
{
    while (xs--)
        Delay_ms(1000);
}
