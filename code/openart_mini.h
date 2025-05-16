#ifndef _openartart_mini_h
#define _openartart_mini_h

#include "zf_common_headfile.h"

#define MTXL_EXP_r                162
#define MTYL_EXP_r                160
#define LETER_NUM_MTXL_EXP_r      120
#define LETER_NUM_MTYL_EXP_r      120

#define MTXL_EXP_l                145
#define MTYL_EXP_l                170
#define LETER_NUM_MTXL_EXP_l      120
#define LETER_NUM_MTYL_EXP_l      120



#define OPENMINIUART_L UART_1
#define OPENMINIUART_R UART_4
void openart_mini(void);

extern uint8 ackFlag;
extern uint8 identifyFlag;
extern float backup_X, backup_Y;
extern uint8_t approachBoxState ;    // 0-未激活 1-正在接近 2-调整姿态
extern int16_t x_error;
extern uint16_t h_error;
extern uint8 zero_flag;
#endif
