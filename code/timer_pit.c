#include "timer_pit.h"

uint32 timeTest_5ms=0;


void timer1_pit_entry(void *parameter)
{
    static uint32 time;
    time++;
    
    if(0 == (time%5))
    {  
      timeTest_5ms++;
    }
    if(0 == (time%2))
    {  
      rt_sem_release(gyroscope_sem);
//      if(anglePIDFlag==1)
//      {
//        Set_Angle();
//        control.Anglecontrol();
//      }
//      if(positionPIDFlag==1)
//      {
//        control.PositionControl();
//      }
//      else
//      {
//        control.RelativePositionControl();
//      }
    }
    
    //控制电机转动
    rt_sem_release(encoder_sem);
}


void timer_pit_init(void)
{
    rt_timer_t timer;
    
    //创建一个定时器 周期运行
    timer = rt_timer_create("timer1", timer1_pit_entry, RT_NULL, 1, RT_TIMER_FLAG_PERIODIC);
    
    //启动定时器
    if(RT_NULL != timer)
    {
        rt_timer_start(timer);
    }    
}
