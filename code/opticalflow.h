#ifndef _opticalflow_h
#define _opticalflow_h

#include "headfile.h"

#define OPTICALFLOW_USART USART_8

extern uint8 opticalflowFlag;
void opticalflow_init(void);
int8 opticalflow_dis_init(void);
#endif