#ifndef _buzzer_h
#define _buzzer_h

#include "zf_common_headfile.h"
#define BUZZER_PIN			B23			// ���������Ϸ�������Ӧ����

extern rt_mailbox_t buzzer_mailbox;//ͨ���Ը����䷢���ʼ�ʵ�ַ���������
int buzzer_init(void);

#endif