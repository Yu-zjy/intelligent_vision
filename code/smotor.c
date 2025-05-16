#include "smotor.h"
#include "zf_common_headfile.h"

#define A_firearms              0//火器
#define B_explosive             1//炸药
#define C_dagger                2//匕首
#define D_spontoon              3//警棍
#define E_fire_axe              4//消防斧
#define F_first_aid_kit         5//急救包
#define G_flashlight            6//手电
#define H_intercom              7//对讲机
#define I_bulletproof_vest      8//防弹背心
#define J_telescope             9//望远镜
#define K_helmet                10//头盔
#define L_fire_engine           11//消防车
#define M_ambulance             12//救护车
#define N_armoured_car          13//装甲车
#define O_motorcycle            14//摩托车


extern CARSTATUS_enum stateTop;
extern uint8 roundabout_type;
extern uint8 start_categorynum;

uint8 carry_flag = 0;//1left,2right
uint8 layflag=0;
int16 servo1_duty, servo2_duty,servo3_duty,servo4_duty,servo5_duty;
uint8 other_picnum[3]={0};
uint8 ELECTROMAGNET_STATE[5][2]={{0,0},{0,0},{0,0},{0,0},{0,0}};
int16 turnstate_l[5]={201,273,345,57,129};
int16 turnstate_r[5]={21,93,165,237,309};
int16 servo_carryangle_l[6][2]={{140,185},{203,188},{190,200},{170,80},{110,5},{135,80}};
int16 servo_layangle_l[6][2]={{150,20},{100,5},{120,40},{165,110},{160,190},{135,80}};
int16 servo_carryangle_r[6][2]={{145,180},{196,180},{185,190},{170,80},{110,5},{125,70}};
int16 servo_layangle_r[6][2]={{150,20},{90,0},{120,40},{140,110},{160,190},{125,70}};
int16 servo_followingangle[6][3];
uint16 servo_followingtime[6]={0};
uint8 smotorProcess_flag=0;
uint8 smotorProcessFinish_flag=0;
uint8 pic_clear=0;
uint8 smotor_terrify=0;
uint8 ser1angle;
uint8 serspecial=0;
uint8 pic_lay_flag=0;
uint32 carrypower;
uint16 servo_time=0;


rt_sem_t smotor_sem;
uint8 transformFromMinorCategory2Side(uint8 category)
{
  if(category<5)
  {
    return 0;
  }
  else if(category<11)
  {
    return 1;
  }
  else
  {
    return 2;
  }
}

extern uint8 mainCategory;//openart判决结果的存储变量


void smotor_entry(void *parameter)
{
    while(1)
    {
        rt_sem_take(smotor_sem,RT_WAITING_FOREVER);
        int16 i;
        if(stateTop==TARGET_CARRY)
        {
          if(roundabout_type==0)
          {
            carrypower=5000;
            ser1angle=transformFromMinorCategory2Side(mainCategory);
            other_picnum[ser1angle]++;
          }
          else
          {
            if(carry_flag==2) carrypower=2000;
            else carrypower=2000;
            ser1angle=4;
            for(i=0;i<5;i++)
            {
              if(ELECTROMAGNET_STATE[i][1]==0)
              {
                ser1angle=4-i;
                ELECTROMAGNET_STATE[i][0]=mainCategory;
                ELECTROMAGNET_STATE[i][1]++;
                break;
              }
              else if(ELECTROMAGNET_STATE[i][0]==mainCategory)
              {
                ser1angle=4-i;
                ELECTROMAGNET_STATE[i][1]++;
                break;
              }
            }
          }
          if(carry_flag==2)
          {
            for(i=0;i<6;i++)
            {
              servo_followingangle[i][0]=turnstate_r[ser1angle];
              servo_followingangle[i][1]=servo_carryangle_r[i][0];
              servo_followingangle[i][2]=servo_carryangle_r[i][1];
            }
          }
          else 
          {
            for(i=0;i<6;i++)
            {
              servo_followingangle[i][0]=turnstate_l[ser1angle];
              servo_followingangle[i][1]=servo_carryangle_l[i][0];
              servo_followingangle[i][2]=servo_carryangle_l[i][1];
            }
          }
        }
        else if(stateTop==TARGET_LAY)
        {
          ser1angle=4;
          for(i=0;i<5;i++)
          {
            if(ELECTROMAGNET_STATE[i][0]==mainCategory&&ELECTROMAGNET_STATE[i][1]!=0)
            {
              ser1angle=4-i;
              break;
            }
          }
          if(ELECTROMAGNET_STATE[4-ser1angle][1]!=0&&ELECTROMAGNET_STATE[4-ser1angle][0]==mainCategory) 
            {
              pic_lay_flag=1;
              carrypower=2000;
              ELECTROMAGNET_STATE[4-ser1angle][1]--;
              if(ELECTROMAGNET_STATE[4-ser1angle][1]==0)
              {
                ELECTROMAGNET_STATE[4-ser1angle][0]=0;
                pic_clear=1;
              }
              else pic_clear=0;
              if(carry_flag==2)
              {
                for(i=0;i<6;i++)
                {
                  servo_followingangle[i][0]=turnstate_r[ser1angle];
                  servo_followingangle[i][1]=servo_layangle_r[i][0];
                  servo_followingangle[i][2]=servo_layangle_r[i][1];
                }
              }
              else
              {
                for(i=0;i<6;i++)
                {
                  servo_followingangle[i][0]=turnstate_l[ser1angle];
                  servo_followingangle[i][1]=servo_layangle_l[i][0];
                  servo_followingangle[i][2]=servo_layangle_l[i][1];
                }
              }
            }
            else pic_lay_flag=0;
        }
        else
        {
          pic_lay_flag=1;
          if(mainCategory==0x11) ser1angle=0;
          else if(mainCategory==0x12) ser1angle=1;
          else ser1angle=2;
          if(other_picnum[ser1angle])
          {
            if(other_picnum[ser1angle]<5) 
            {
              other_picnum[ser1angle]--;
              carrypower=2000;
            }
            else
            {
              other_picnum[ser1angle]-=2;
              carrypower=4000;
            }
            if(other_picnum[ser1angle]==0) 
            {
              if(other_picnum[0]==0&&other_picnum[1]==0&&other_picnum[2]==0) start_categorynum=3;
              pic_clear=1;
            }
            else pic_clear=0;
          }
          else pic_lay_flag=0;
          if(carry_flag==2)
          {
            for(i=0;i<6;i++)
            {
              servo_followingangle[i][0]=turnstate_r[ser1angle];
              servo_followingangle[i][1]=servo_layangle_r[i][0];
              servo_followingangle[i][2]=servo_layangle_r[i][1];
            }  
          }
          else
          {
            for(i=0;i<6;i++)
            {
              servo_followingangle[i][0]=turnstate_l[ser1angle];
              servo_followingangle[i][1]=servo_layangle_l[i][0];
              servo_followingangle[i][2]=servo_layangle_l[i][1];
            }   
          }
        }
        if(pic_lay_flag||stateTop==TARGET_CARRY)
        {
          uint8 time_use_10ms=0;
          smotorProcessFinish_flag=0;
          smotor_terrify=0;
          if(carry_flag==2) 
          {
            smotorProcess_flag=2;
            servo_followingtime[0]=abs(servo_followingangle[0][0]-servo1_duty)/10+1;
            time_use_10ms=abs(servo_followingangle[0][1]-servo4_duty)/10+1;
            if(time_use_10ms>servo_followingtime[0]) servo_followingtime[0]=time_use_10ms;
            time_use_10ms=abs(servo_followingangle[0][2]-servo5_duty)/10+1;
            if(time_use_10ms>servo_followingtime[0]) servo_followingtime[0]=time_use_10ms;
            for(i=1;i<6;i++)
            {
              servo_followingtime[i]=abs(servo_followingangle[i][0]-servo_followingangle[i-1][0])/10+1;
              time_use_10ms=abs(servo_followingangle[i][1]-servo_followingangle[i-1][1])/10+1;
              if(time_use_10ms>servo_followingtime[i]) servo_followingtime[i]=time_use_10ms;
              time_use_10ms=abs(servo_followingangle[i][2]-servo_followingangle[i-1][2])/10+1;
              if(time_use_10ms>servo_followingtime[i]) servo_followingtime[i]=time_use_10ms;
            }
          }
          else 
          {
            smotorProcess_flag=1;
            servo_followingtime[0]=abs(servo_followingangle[0][0]-servo1_duty)/10+1;
            time_use_10ms=abs(servo_followingangle[0][1]-servo2_duty)/10+1;
            if(time_use_10ms>servo_followingtime[0]) servo_followingtime[0]=time_use_10ms;
            time_use_10ms=abs(servo_followingangle[0][2]-servo3_duty)/10+1;
            if(time_use_10ms>servo_followingtime[0]) servo_followingtime[0]=time_use_10ms;
            for(i=1;i<6;i++)
            {
              servo_followingtime[i]=abs(servo_followingangle[i][0]-servo_followingangle[i-1][0])/10+1;
              time_use_10ms=abs(servo_followingangle[i][1]-servo_followingangle[i-1][1])/10+1;
              if(time_use_10ms>servo_followingtime[i]) servo_followingtime[i]=time_use_10ms;
              time_use_10ms=abs(servo_followingangle[i][2]-servo_followingangle[i-1][2])/10+1;
              if(time_use_10ms>servo_followingtime[i]) servo_followingtime[i]=time_use_10ms;
            }
          }
          pic_lay_flag=0;
        }
        else
        {
          pic_clear=1;
          smotorProcessFinish_flag=1;
          smotor_terrify=0;
          smotorProcess_flag=0;
        }
    }
}

//void timer_smotor_entry(void *parameter)
//{
//  if(smotorProcess_flag==1)
//  {
//    if(abs(servo1_duty - servo_followingangle[smotor_terrify][0]) >= 1)
//    {
//      if(servo_followingangle[smotor_terrify][0]>servo1_duty) servo1_duty++;
//      else servo1_duty--;
//    }
//    else servo1_duty = servo_followingangle[smotor_terrify][0];
//    
//    if(abs(servo2_duty - servo_followingangle[smotor_terrify][1]) >= 1) 
//    {
//      if(servo_followingangle[smotor_terrify][1]>servo2_duty) servo2_duty++;
//      else servo2_duty--;
//    }
//    else servo2_duty = servo_followingangle[smotor_terrify][1];
//    
//    if(abs(servo3_duty - servo_followingangle[smotor_terrify][2]) >= 1) 
//    {
//      if(servo_followingangle[smotor_terrify][2]>servo3_duty) servo3_duty++;
//      else servo3_duty--;
//    }
//    else servo3_duty = servo_followingangle[smotor_terrify][2];
//    
//    pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY_360((uint16)servo1_duty));
//    pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY_270((uint16)servo2_duty));
//    pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY_270((uint16)servo3_duty));
//    if(abs(servo1_duty - servo_followingangle[smotor_terrify][0])<1&&abs(servo2_duty - servo_followingangle[smotor_terrify][1])<1&&abs(servo3_duty - servo_followingangle[smotor_terrify][2])<1)
//    {
//      smotor_terrify++;
//      if(smotor_terrify==1) pwm_set_duty(ELECTROMAGNET_0,carrypower);
////      else if(smotor_terrify==4) pwm_set_duty(ELECTROMAGNET_0,carrypower);
//      else if(smotor_terrify==5) 
//      {
//        pwm_set_duty(ELECTROMAGNET_0,0);
//        smotorProcessFinish_flag=1;
//      }
//      else if(smotor_terrify==6)
//      {
//        smotor_terrify=0;
//        smotorProcess_flag=0;
//      }
//    }   
//  }
//  else if(smotorProcess_flag==2)
//  {
//    if(abs(servo1_duty - servo_followingangle[smotor_terrify][0]) >= 1)
//    {
//      if(servo_followingangle[smotor_terrify][0]>servo1_duty) servo1_duty++;
//      else servo1_duty--;
//    }
//    else servo1_duty = servo_followingangle[smotor_terrify][0];
//    
//    if(abs(servo4_duty - servo_followingangle[smotor_terrify][1]) >= 1) 
//    {
//      if(servo_followingangle[smotor_terrify][1]>servo4_duty) servo4_duty++;
//      else servo4_duty--;
//    }
//    else servo4_duty = servo_followingangle[smotor_terrify][1];
//    
//    if(abs(servo5_duty - servo_followingangle[smotor_terrify][2]) >= 1) 
//    {
//      if(servo_followingangle[smotor_terrify][2]>servo5_duty) servo5_duty++;
//      else servo5_duty--;
//    }
//    else servo5_duty = servo_followingangle[smotor_terrify][2];
//    
//    pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY_360((uint16)servo1_duty));
//    pwm_set_duty(SERVO_MOTOR_PWM4, (uint32)SERVO_MOTOR_DUTY_270((uint16)servo4_duty));
//    pwm_set_duty(SERVO_MOTOR_PWM5, (uint32)SERVO_MOTOR_DUTY_270((uint16)servo5_duty));
//    if(abs(servo1_duty - servo_followingangle[smotor_terrify][0])<1&&abs(servo4_duty - servo_followingangle[smotor_terrify][1])<1&&abs(servo5_duty - servo_followingangle[smotor_terrify][2])<1)
//    {
//      smotor_terrify++;
//      if(smotor_terrify==1) pwm_set_duty(ELECTROMAGNET_1,carrypower);
////      else if(smotor_terrify==4) pwm_set_duty(ELECTROMAGNET_1,carrypower);
//      else if(smotor_terrify==5) 
//      {
//        pwm_set_duty(ELECTROMAGNET_1,0);
//        smotorProcessFinish_flag=1;
//      }
//      else if(smotor_terrify==6)
//      {
//        smotor_terrify=0;
//        smotorProcess_flag=0;
//      }
//    }   
//  }
//}

void timer_smotor_entry(void *parameter)
{
  if(smotorProcess_flag==1)
  {
    pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY_360((uint16)servo_followingangle[smotor_terrify][0]));
    pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY_270((uint16)servo_followingangle[smotor_terrify][2]));
    pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY_270((uint16)servo_followingangle[smotor_terrify][1]));
    servo1_duty=servo_followingangle[smotor_terrify][0];
    servo2_duty=servo_followingangle[smotor_terrify][1];
    servo3_duty=servo_followingangle[smotor_terrify][2];
    if(servo_time>servo_followingtime[smotor_terrify])
    {
      servo_time=0;
      servo_followingtime[smotor_terrify]=0;
      smotor_terrify++;
      if(smotor_terrify==1) pwm_set_duty(ELECTROMAGNET_0,carrypower);
      else if(smotor_terrify==5) 
      {
        pwm_set_duty(ELECTROMAGNET_0,0);
        smotorProcessFinish_flag=1;
      }
      else if(smotor_terrify==6)
      {
        smotor_terrify=0;
        smotorProcess_flag=0;
      }   
    }
    else servo_time++;
  }
  else if(smotorProcess_flag==2)
  {
    pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY_360((uint16)servo_followingangle[smotor_terrify][0]));
    pwm_set_duty(SERVO_MOTOR_PWM5, (uint32)SERVO_MOTOR_DUTY_270((uint16)servo_followingangle[smotor_terrify][2]));
    pwm_set_duty(SERVO_MOTOR_PWM4, (uint32)SERVO_MOTOR_DUTY_270((uint16)servo_followingangle[smotor_terrify][1]));
    servo1_duty=servo_followingangle[smotor_terrify][0];
    servo4_duty=servo_followingangle[smotor_terrify][1];
    servo5_duty=servo_followingangle[smotor_terrify][2];
    if(servo_time>servo_followingtime[smotor_terrify])
    {
      servo_time=0;
      servo_followingtime[smotor_terrify]=0;
      smotor_terrify++;
      if(smotor_terrify==1) pwm_set_duty(ELECTROMAGNET_1,carrypower);
      else if(smotor_terrify==5) 
      {
        pwm_set_duty(ELECTROMAGNET_1,0);
        if(roundabout_type==0||stateTop!=TARGET_CARRY) smotorProcessFinish_flag=1;
      }
      else if(smotor_terrify==6)
      {
        smotor_terrify=0;
        smotorProcess_flag=0;
        if(roundabout_type&&stateTop==TARGET_CARRY) smotorProcessFinish_flag=1;
      }   
    }
    else servo_time++;
  }
}


int smotor_init(void)
{
    rt_thread_t tid;
    rt_timer_t timer_smotor;
    
    servo1_duty = 182;servo2_duty = 135;
    pwm_init(SERVO_MOTOR_PWM1, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY_360(servo1_duty));
    pwm_init(SERVO_MOTOR_PWM2, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY_270(servo2_duty));
    servo3_duty=80;servo4_duty=125;
    pwm_init(SERVO_MOTOR_PWM3, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY_270(servo3_duty));
    pwm_init(SERVO_MOTOR_PWM4, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY_270(servo4_duty));
    servo5_duty=70;
    pwm_init(SERVO_MOTOR_PWM5, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY_270(servo5_duty));
    
    pwm_init(ELECTROMAGNET_0, 20000,0);
    pwm_init(ELECTROMAGNET_1, 20000,0);
    //创建线程 优先级3
    smotor_sem = rt_sem_create("smotor_sem",0,RT_IPC_FLAG_PRIO);
    tid = rt_thread_create("smotor", smotor_entry, RT_NULL, 512, 2, 3);
    
    //启动线程
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
    else													// 线程创建失败
    {
        rt_kprintf("smotorR thread create ERROR.\n");
        return -1;
    }
    
    timer_smotor = rt_timer_create("timer_smotor", timer_smotor_entry, RT_NULL, 50, RT_TIMER_FLAG_PERIODIC);
    if(RT_NULL != timer_smotor) 
    {
        rt_timer_start(timer_smotor);
    }
    else													// 时钟创建失败
    {
        rt_kprintf("timer_smotor create ERROR.\n");
        return -1;
    }
    return 0;
}