#ifndef _display_h
#define _display_h

#include "zf_common_headfile.h"
//信号量创建
extern rt_sem_t display_sem;

void display_init(void);
#endif