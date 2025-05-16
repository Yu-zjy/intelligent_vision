#include "nvic.h"

void Priority_Init(void)
{
    pit_ms_init(PIT_CH0,2);                    //初始化pit外设
                                            //初始化pit通道0 周期2ms  用于陀螺仪姿态获取
    NVIC_SetPriority(PIT_IRQn,1);  //设置中断优先级 范围0-15 越小优先级越高 四路PIT共用一个PIT中断函数     
    
    
    pit_ms_init(PIT_CH1,5);  //初始化pit通道1 周期5ms  用于各电机pid
    NVIC_SetPriority(PIT_IRQn,2);  //设置中断优先级 范围0-15 越小优先级越高 四路PIT共用一个PIT中断函数   
}









