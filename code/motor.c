#include "motor.h"

//纯支持函数，没有进程，被control进程调用

int motor_init(void)
{
   //20K 的PWM频率 
   pwm_init(pwmRightFront_forward, 20000, 0);
   pwm_init(pwmRightFront_backward, 20000, 0);
   pwm_init(pwmLeftFront_forward, 20000, 0);
   pwm_init(pwmLeftFront_backward, 20000, 0);
   
   pwm_init(pwmRear_forward, 20000, 0);
   pwm_init(pwmRear_backward, 20000, 0);

   return 0;
}                              


/// 要改
int MotorPwm[6] = {0};
//每个电机两路PWM，分别控制正转和反转  这个.c文件基本上只用调用这个和初始化就行了

void MotorPwmFlash(int *MotorPwm)
{
  pwm_set_duty(pwmLeftFront_forward, MotorPwm[0]);//左前轮正转
  pwm_set_duty(pwmLeftFront_backward, MotorPwm[1]);//左前轮反转
  pwm_set_duty(pwmRightFront_forward, MotorPwm[2]);//右前轮正转
  pwm_set_duty(pwmRightFront_backward, MotorPwm[3]);//右前轮反转
  pwm_set_duty(pwmRear_forward, MotorPwm[4]);//后轮正转    
  pwm_set_duty(pwmRear_backward, MotorPwm[5]);//后轮反转
}


int16 PWM_Limit(float  PWM, int16 max)
{
    if(PWM > -max && PWM < max)
        return (int16)PWM;
    else if( PWM >=  max)
        PWM = max;
    else
        PWM = -max;
    return (int16)PWM;
}
