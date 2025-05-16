#include "motor.h"

//��֧�ֺ�����û�н��̣���control���̵���

int motor_init(void)
{
   //20K ��PWMƵ�� 
   pwm_init(pwmRightFront_forward, 20000, 0);
   pwm_init(pwmRightFront_backward, 20000, 0);
   pwm_init(pwmLeftFront_forward, 20000, 0);
   pwm_init(pwmLeftFront_backward, 20000, 0);
   
   pwm_init(pwmRear_forward, 20000, 0);
   pwm_init(pwmRear_backward, 20000, 0);

   return 0;
}                              


/// Ҫ��
int MotorPwm[6] = {0};
//ÿ�������·PWM���ֱ������ת�ͷ�ת  ���.c�ļ�������ֻ�õ�������ͳ�ʼ��������

void MotorPwmFlash(int *MotorPwm)
{
  pwm_set_duty(pwmLeftFront_forward, MotorPwm[0]);//��ǰ����ת
  pwm_set_duty(pwmLeftFront_backward, MotorPwm[1]);//��ǰ�ַ�ת
  pwm_set_duty(pwmRightFront_forward, MotorPwm[2]);//��ǰ����ת
  pwm_set_duty(pwmRightFront_backward, MotorPwm[3]);//��ǰ�ַ�ת
  pwm_set_duty(pwmRear_forward, MotorPwm[4]);//������ת    
  pwm_set_duty(pwmRear_backward, MotorPwm[5]);//���ַ�ת
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
