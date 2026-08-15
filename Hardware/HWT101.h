#ifndef __DATA_H
#define __DATA_H

extern float current_yaw;
void Hwt101_Dma_Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendHex(uint8_t *Data, uint16_t Length);
void Serial_SendString(char *String);
uint32_t Serial_Pow(uint32_t X, uint32_t Y);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);
float calculate_shortest_path_error(float error);
extern volatile float Current_World_Yaw;
#endif
