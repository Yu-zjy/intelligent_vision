#ifndef _button_h
#define _button_h

#include "headfile.h"

#define KEY_1   C12//C14//C11	// ���������ϰ�����Ӧ����
#define KEY_2   C13//C0	// ���������ϰ�����Ӧ����
#define KEY_3   C14//D26	// ���������ϰ�����Ӧ����
#define KEY_4   D26//C12	// ���������ϰ�����Ӧ����
#define KEY_5   C15//C5	// ���������ϰ�����Ӧ����
#define KEY_6   D27	// ���������ϰ�����Ӧ����
//#define KEY_7   C11	// ���������ϰ�����Ӧ����
#define GPIO_KEY_CONFIG         SPEED_100MHZ | DSE_R0 | PULLDOWN_100K | PULL_EN	//�궨��GPIO���ŵ�Ĭ�����ã����ڳ�ʼ��GPIOʱ������д�����������Ҫ���������������޸�

int button_init(void);

#endif