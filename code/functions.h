#ifndef _functions_h
#define _functions_h

#include "zf_common_headfile.h"

//FreeCars��λ�� ����������ʾ�� ����ͨ������������λ�����øı�
#define UartDataNum 17//��λ������ͨ����������������λ������һ��

void freeCars_osc(void);
void uint2Byte(float *target, int8 *buf, int8 beg4);
void virtual_osc (void);
float Limit_float(float  value, float max);
float Limit1_float(float  value, float max, float min);
float Slope_Calculate(uint8 begin,uint8 end,int16 *p);
//float variance(uint8 begin,uint8 end,int16 *p);
int16 abs16 (int16 x);

#endif