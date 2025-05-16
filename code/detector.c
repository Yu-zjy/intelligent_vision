#include "detector.h"

void detector_init(void);

rt_sem_t detector_sem;

extern uint8 road_type;
extern uint8 move_state;

uint8 left_last=1,right_last=1,left_now=1,right_now=1;
uint8 find_pic=0;//1左边，2右边

void detector_entry(void *parameter)
{
  while(1)
  {
    rt_sem_take(detector_sem, RT_WAITING_FOREVER);
    if((road_type==STRAIGHT||road_type==CURVE)&&find_pic==0)
    {
      left_now=gpio_get_level(DETECTOR_PIN1);
      right_now=gpio_get_level(DETECTOR_PIN2);
      if((!left_now&&left_last)||(!right_now&&right_last))//下降沿减速
      {
        if(!left_now) find_pic=1;
        else find_pic=2;
        move_state=4;
      }
      left_last=left_now;
      right_last=right_now;
    }
    rt_sem_release(control_sem);
  }
}

void detector_init(void)
{
  rt_thread_t tid;
  gpio_init(DETECTOR_PIN1,GPI,1,FAST_GPI_PULL_UP);
  gpio_init(DETECTOR_PIN2,GPI,1,FAST_GPI_PULL_UP);
  
  detector_sem = rt_sem_create("detector_sem", 0, RT_IPC_FLAG_PRIO);
       
  tid = rt_thread_create("detector", detector_entry, RT_NULL, 512, 6, 10);//优先级很高，保证检测图片即使响应
  
  //启动显示线程
  if(RT_NULL != tid)
  {
      rt_thread_startup(tid);
  }
  else													// 线程创建失败
  {
      rt_kprintf("image thread create ERROR.\n");
      return ;
  }
}