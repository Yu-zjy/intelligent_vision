#ifndef _WDOG
#define _WDOG
#include "headfile.h"

#define MY_WDOG_BASE WDOG1
#define MY_WDOG_callback                WDOG1_IRQHandler
/*时间的计算是 设置的数加1乘以0.5 seconds*/
//3 sec 中断一次
#define TIMEOUT_VALUE  0x7U /* Timeout value is 4 sec. */
#define INT_BEFORETIMREOUT 0x1U /* Interrupt occurred 1 sec before WDOG timeout. */

void wdogInterrupt_init(void);


#endif