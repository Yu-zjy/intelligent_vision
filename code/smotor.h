#ifndef _smotor_h_
#define _smotor_h_

#include "zf_common_headfile.h"


#define SERVO_MOTOR_PWM1                PWM2_MODULE1_CHA_C8                   // 定义主板上舵机对应引脚
#define SERVO_MOTOR_PWM2                PWM2_MODULE0_CHA_C6                   // 定义主板上舵机对应引脚
#define SERVO_MOTOR_PWM3                PWM2_MODULE0_CHB_C7
#define SERVO_MOTOR_PWM4                PWM2_MODULE1_CHB_C9
#define SERVO_MOTOR_PWM5                PWM2_MODULE2_CHA_C10
#define SERVO_MOTOR_FREQ                125
#define ELECTROMAGNET_0                 PWM2_MODULE3_CHA_D2
#define ELECTROMAGNET_1                 PWM2_MODULE3_CHB_D3

#define SERVO_MOTOR_DUTY_180(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))
#define SERVO_MOTOR_DUTY_270(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/135.0))
#define SERVO_MOTOR_DUTY_360(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/180.0))

extern uint8 carry_flag;
extern uint8 layflag;
extern rt_sem_t smotor_sem;

uint8 transformFromMinorCategory2Side(uint8 category);
void smotor_laydown(rt_int32_t ms);
void servo_slow_ctrl(uint16 _servo1_angle, uint16 _servo2_angle, float _step_count);
void servo_trans(void);
void servo_basket(void);
void pick(void);
void servo_sleep(void);
int smotor_init(void); 

#endif