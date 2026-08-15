#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

/* F4移植: 3个舵机
 *   Servo_CH1: TIM2_CH1 → PA5
 *   Servo_CH2: TIM2_CH2 → PB3 (需释放JTAG)
 *   Servo_CH3: TIM9_CH2 → PE6
 */

/* ========== 原有函数（保持不变） ========== */
void Servo_CH1_Init(void);
void Servo_CH2_Init(void);
void Servo_CH3_Init(void);

void Servo_SetAngle_CH1(float Angle);
void Servo_SetAngle_CH2(float Angle);
void Servo_SetAngle_CH3(float Angle);

/* ========== S型加减速扩展 ========== */
/* 说明：时间基准使用 System_Tick_10ms（10ms递增），
 *       与 MC.c 的 Chassis_Core 完全对齐 */

typedef struct {
    float    start_angle;   // 本次运动起点（运动中不变）
    float    target_angle;  // 目标角度
    uint8_t  is_moving;     // 0=空闲, 1=运动中
    uint32_t start_tick;    // 运动开始时的 System_Tick_10ms 值
    uint32_t duration;      // 过渡总时长（单位：System_Tick_10ms 的 tick 数，1 tick = 10ms）
} ServoMotion_t;

extern ServoMotion_t gServo1_Motion;  // PA5
extern ServoMotion_t gServo2_Motion;  // PB3
extern ServoMotion_t gServo3_Motion;  // PE6

/* 平滑设置角度
 * angle:       目标角度 (0~180°)
 * duration_10ms: 过渡时长，单位 10ms。例如 35 表示 350ms */
void Servo_SmoothSetAngle_CH1(float angle, uint32_t duration_10ms);  //爪子
void Servo_SmoothSetAngle_CH2(float angle, uint32_t duration_10ms);  //转盘
void Servo_SmoothSetAngle_CH3(float angle, uint32_t duration_10ms);  //皮带

/* 核心更新函数：放在你的系统定时器中断（10ms）中调用 */
void Servo_SmoothUpdate(void);

#endif
