#ifndef _motor_h
#define _motor_h

#include "zf_common_headfile.h"
//1
#define pwmLeftFront_forward            PWM2_MODULE1_CHB_C9
#define pwmLeftFront_backward           PWM2_MODULE1_CHA_C8
#define pwmRightFront_forward           PWM2_MODULE0_CHB_C7
#define pwmRightFront_backward          PWM2_MODULE0_CHA_C6
#define pwmRear_forward                 PWM2_MODULE2_CHA_C10
#define pwmRear_backward                PWM2_MODULE3_CHA_D2


extern int MotorPwm[6];

void MotorPwmFlash(int *MotorPwm);
int16 PWM_Limit(float  PWM, int16 max);
int motor_init(void);
    
#endif