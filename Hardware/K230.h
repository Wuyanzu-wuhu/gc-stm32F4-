#ifndef __K230_H
#define __K230_H


/* ===== 全局视觉数据 ===== */
extern volatile int16_t  g_visual_dx;         /* X方向偏差 (正=偏右, 负=偏左) */
extern volatile int16_t  g_visual_dy;         /* Y方向偏差 (正=偏下, 负=偏上) */
extern volatile uint8_t  g_visual_stop_flag;  /* 1=到位/停止 (兼容FSM代码) */

void  K230_Dma_Serial_Init(void);
void  K230_Dma_SendHex(uint8_t *Data, uint16_t Length);
void  Grap1_Wei_Tiao_Function(float g_target_yaw);
void Grap2_Wei_Tiao_Function(float g_target_yaw);
void  K230_SendTaskCmd(uint8_t cmd);
void  K230_SendQR(const char *qr);

#endif
