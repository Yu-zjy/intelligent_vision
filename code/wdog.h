#ifndef _WDOG
#define _WDOG
#include "headfile.h"

#define MY_WDOG_BASE WDOG1
#define MY_WDOG_callback                WDOG1_IRQHandler
/*ʱ��ļ����� ���õ�����1����0.5 seconds*/
//3 sec �ж�һ��
#define TIMEOUT_VALUE  0x7U /* Timeout value is 4 sec. */
#define INT_BEFORETIMREOUT 0x1U /* Interrupt occurred 1 sec before WDOG timeout. */

void wdogInterrupt_init(void);


#endif