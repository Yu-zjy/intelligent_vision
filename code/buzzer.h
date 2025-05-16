#ifndef _buzzer_h
#define _buzzer_h

#include "zf_common_headfile.h"
#define BUZZER_PIN			B23			// 定义主板上蜂鸣器对应引脚

extern rt_mailbox_t buzzer_mailbox;//通过对该邮箱发送邮件实现蜂鸣器控制
int buzzer_init(void);

#endif