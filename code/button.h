#ifndef _button_h
#define _button_h

#include "headfile.h"

#define KEY_1   C12//C14//C11	// 定义主板上按键对应引脚
#define KEY_2   C13//C0	// 定义主板上按键对应引脚
#define KEY_3   C14//D26	// 定义主板上按键对应引脚
#define KEY_4   D26//C12	// 定义主板上按键对应引脚
#define KEY_5   C15//C5	// 定义主板上按键对应引脚
#define KEY_6   D27	// 定义主板上按键对应引脚
//#define KEY_7   C11	// 定义主板上按键对应引脚
#define GPIO_KEY_CONFIG         SPEED_100MHZ | DSE_R0 | PULLDOWN_100K | PULL_EN	//宏定义GPIO引脚的默认配置，便于初始化GPIO时快速填写参数，如果需要其他参数可自行修改

int button_init(void);

#endif