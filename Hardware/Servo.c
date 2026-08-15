/**
 * Servo.c - 舵机PWM驱动 (STM32F407VGT6)
 *
 * 从 F1 (TIM8 PC6/7/8/9 四路) 移植:
 *   Servo_CH1: TIM2_CH1 → PA5  (AF1)    - TIM2在APB1: 2×42=84MHz, /84=1MHz
 *   Servo_CH2: TIM2_CH2 → PB3  (AF1)    - TIM2在APB1: 2×42=84MHz, /84=1MHz
 *   Servo_CH3: TIM9_CH2 → PE6  (AF3)    - TIM9在APB2: 2×84=168MHz, /168=1MHz
 *
 * 舵机参数: 50Hz(20ms), 0.5~2.5ms → 0~180°
 */
#include "stm32f4xx.h"
#include <math.h>
#include <Servo.h>
#define PI 3.14159265f

/* ========== 外部引用：系统10ms tick（与 MC.c 共用） ========== */
extern uint32_t System_Tick_10ms;

/* ========== 运动状态实例 ========== */
ServoMotion_t gServo1_Motion = {0};
ServoMotion_t gServo2_Motion = {0};
ServoMotion_t gServo3_Motion = {0};

/* ==================== 原有 PWM 初始化 ==================== */

/* ========== Servo_CH1: TIM2_CH1 → PA5 ========== */
void Servo_CH1_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_TIM2);

    TIM_Cmd(TIM2, ENABLE);
}

/* ========== Servo_CH2: TIM2_CH2 → PB3 ========== */
void Servo_CH2_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);

    TIM_Cmd(TIM2, ENABLE);
}

/* ========== Servo_CH3: TIM9_CH2 → PE6 ========== */
void Servo_CH3_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
    TIM_InternalClockConfig(TIM9);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 168 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM9, &TIM_TimeBaseInitStructure);

    TIM_ClearFlag(TIM9, TIM_FLAG_Update);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC2Init(TIM9, &TIM_OCInitStructure);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);

    TIM_Cmd(TIM9, ENABLE);
}

/* ==================== 原有角度设置 ==================== */

void Servo_SetAngle_CH1(float Angle)
{
    TIM_SetCompare1(TIM2, (uint16_t)(Angle / 180.0f * 2000 + 500));
}

void Servo_SetAngle_CH2(float Angle)
{
    TIM_SetCompare2(TIM2, (uint16_t)(Angle / 270.0f * 2000 + 500));
}

void Servo_SetAngle_CH3(float Angle)
{
    TIM_SetCompare2(TIM9, (uint16_t)(Angle / 270.0f * 2000 + 500));
}

/* ==================== 去掉函数指针，直接展开 ==================== */

void Servo_SmoothUpdate(void)
{
    uint32_t elapsed;
    float ratio, out;

    /* ---------- CH1 ---------- */
    if (gServo1_Motion.is_moving) {
        elapsed = System_Tick_10ms - gServo1_Motion.start_tick;
        if (elapsed >= gServo1_Motion.duration) {
            Servo_SetAngle_CH1(gServo1_Motion.target_angle);
            gServo1_Motion.start_angle = gServo1_Motion.target_angle;
            gServo1_Motion.is_moving = 0;
        } else {
            ratio = 0.5f - 0.5f * cosf(PI * (float)elapsed / (float)gServo1_Motion.duration);
            out = gServo1_Motion.start_angle + (gServo1_Motion.target_angle - gServo1_Motion.start_angle) * ratio;
            Servo_SetAngle_CH1(out);
        }
    }

    /* ---------- CH2 ---------- */
    if (gServo2_Motion.is_moving) {
        elapsed = System_Tick_10ms - gServo2_Motion.start_tick;
        if (elapsed >= gServo2_Motion.duration) {
            Servo_SetAngle_CH2(gServo2_Motion.target_angle);
            gServo2_Motion.start_angle = gServo2_Motion.target_angle;
            gServo2_Motion.is_moving = 0;
        } else {
            ratio = 0.5f - 0.5f * cosf(PI * (float)elapsed / (float)gServo2_Motion.duration);
            out = gServo2_Motion.start_angle + (gServo2_Motion.target_angle - gServo2_Motion.start_angle) * ratio;
            Servo_SetAngle_CH2(out);
        }
    }

    /* ---------- CH3 ---------- */
    if (gServo3_Motion.is_moving) {
        elapsed = System_Tick_10ms - gServo3_Motion.start_tick;
        if (elapsed >= gServo3_Motion.duration) {
            Servo_SetAngle_CH3(gServo3_Motion.target_angle);
            gServo3_Motion.start_angle = gServo3_Motion.target_angle;
            gServo3_Motion.is_moving = 0;
        } else {
            ratio = 0.5f - 0.5f * cosf(PI * (float)elapsed / (float)gServo3_Motion.duration);
            out = gServo3_Motion.start_angle + (gServo3_Motion.target_angle - gServo3_Motion.start_angle) * ratio;
            Servo_SetAngle_CH3(out);
        }
    }
}

/*
 * 函数简介：所有舵机S型加减速更新入口
 * 参数说明：无
 * 返回类型：无
 * 备注：放在 System_Tick_10ms 所在的中断里调用（10ms周期）
 */

/*
 * 函数简介：CH1舵机平滑设置角度
 * 参数说明：angle         - 目标角度 (0~180°)
 *           duration_10ms - 过渡时长，单位10ms（如35=350ms）
 * 返回类型：无
 * 备注：以当前 target_angle 作为起点，启动 S 型过渡
 */
void Servo_SmoothSetAngle_CH1(float angle, uint32_t duration_10ms)
{
    gServo1_Motion.start_angle = gServo1_Motion.target_angle;
    gServo1_Motion.target_angle = angle;
    gServo1_Motion.duration = duration_10ms;
    gServo1_Motion.start_tick = System_Tick_10ms;
    gServo1_Motion.is_moving = 1;
}

void Servo_SmoothSetAngle_CH2(float angle, uint32_t duration_10ms)
{
    gServo2_Motion.start_angle = gServo2_Motion.target_angle;
    gServo2_Motion.target_angle = angle;
    gServo2_Motion.duration = duration_10ms;
    gServo2_Motion.start_tick = System_Tick_10ms;
    gServo2_Motion.is_moving = 1;
}

void Servo_SmoothSetAngle_CH3(float angle, uint32_t duration_10ms)
{
    gServo3_Motion.start_angle = gServo3_Motion.target_angle;
    gServo3_Motion.target_angle = angle;
    gServo3_Motion.duration = duration_10ms;
    gServo3_Motion.start_tick = System_Tick_10ms;
    gServo3_Motion.is_moving = 1;
}
