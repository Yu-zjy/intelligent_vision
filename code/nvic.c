#include "nvic.h"

void Priority_Init(void)
{
    pit_ms_init(PIT_CH0,2);                    //��ʼ��pit����
                                            //��ʼ��pitͨ��0 ����2ms  ������������̬��ȡ
    NVIC_SetPriority(PIT_IRQn,1);  //�����ж����ȼ� ��Χ0-15 ԽС���ȼ�Խ�� ��·PIT����һ��PIT�жϺ���     
    
    
    pit_ms_init(PIT_CH1,5);  //��ʼ��pitͨ��1 ����5ms  ���ڸ����pid
    NVIC_SetPriority(PIT_IRQn,2);  //�����ж����ȼ� ��Χ0-15 ԽС���ȼ�Խ�� ��·PIT����һ��PIT�жϺ���   
}









