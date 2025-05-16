#include "buzzer.h"
#include "button.h"

//debug
#define KEYtoIDENTFY

# define SHRIEK 10
# define RAPAID 8
# define CLEAR 5


//����״̬����
uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
uint8 key4_status = 1;
uint8 key5_status = 1;
uint8 key6_status = 1;

//��һ�ο���״̬����
uint8 key1_last_status;
uint8 key2_last_status;
uint8 key3_last_status;
uint8 key4_last_status;
uint8 key5_last_status;
uint8 key6_last_status;

//�����ź���
rt_sem_t key1_sem;
rt_sem_t key2_sem;
rt_sem_t key3_sem;
rt_sem_t key4_sem;
rt_sem_t key5_sem;
rt_sem_t key6_sem;

extern CARSTATUS_enum stateTop;
//������ں���
void key1_entry(void *parameter)
{
  while(1)
  {
    rt_sem_take(key1_sem, RT_WAITING_FOREVER);
    
    rt_mb_send(buzzer_mailbox, CLEAR);
    menu_pfc[menu_level](4);
    if( positionPIDFlag == 1)
    {
      //����opneart����
      uart_putchar(OPENMINIUART, 0xff);
      uart_putchar(OPENMINIUART, 0xAA);
      uart_putchar(OPENMINIUART, 0xCC);
      rt_mb_send(buzzer_mailbox, 100);
    }
  }
}
extern unsigned char csimenu_flag[4];//����ͷ��־λ,������ͨ��UI��
extern uint8 THRESHOLD;
void key2_entry(void *parameter)
{
  while(1)
  {
    rt_sem_take(key2_sem, RT_WAITING_FOREVER);
    
    rt_mb_send(buzzer_mailbox, CLEAR);
    menu_pfc[menu_level](5);//ȷ������
  }
}

void key3_entry(void *parameter)
{
  while(1)
  {
    rt_sem_take(key3_sem, RT_WAITING_FOREVER);
    
    rt_mb_send(buzzer_mailbox, SHRIEK);
    menu_pfc[menu_level](6);
  }
}
extern int terify_flag;
extern uint8 ackFlag;
extern uint8 identifyFlag;
void key4_entry(void *parameter)
{
  while(1)
  {
    rt_sem_take(key4_sem, RT_WAITING_FOREVER);    
    rt_mb_send(buzzer_mailbox, RAPAID);
    menu_pfc[menu_level](1);
#ifdef KEYtoIDENTFY
    if( positionPIDFlag == 1)
    {
      stateTop=TARGET_IDENTIFY;
      terify_flag=1;
      ackFlag=0;
      identifyFlag=0;
    }
#endif
    if(csimenu_flag[2]==1)//��ʾ�����
    {
      THRESHOLDforJunmpPoint = (THRESHOLDforJunmpPoint<=0)?0:THRESHOLDforJunmpPoint-1;
    }
  }
}
void key5_entry(void *parameter)//up key
{
  while(1)
  {
    rt_sem_take(key5_sem, RT_WAITING_FOREVER);    
    rt_mb_send(buzzer_mailbox, SHRIEK);
    menu_pfc[menu_level](2);
    if(csimenu_flag[2]==1)//��ʾ�����
    {
      THRESHOLDforJunmpPoint = (THRESHOLDforJunmpPoint>=255)?255:THRESHOLDforJunmpPoint+1;
    }
  }
}

void key6_entry(void *parameter)
{
  while(1)
  {
    rt_sem_take(key6_sem, RT_WAITING_FOREVER);
    
    rt_mb_send(buzzer_mailbox, RAPAID);
    menu_pfc[menu_level](3);
    if(csimenu_flag[0] == 1)
    {
      THRESHOLD = (THRESHOLD == THRESHOLDFORPICTURE)? THRESHOLDFORBACK:THRESHOLDFORPICTURE;
    }

  }
}

//void key7_entry(void *parameter)
//{
//  while(1)
//  {
//    rt_sem_take(key7_sem, RT_WAITING_FOREVER);
//#ifdef KEYtoIDENTFY    
//    rt_mb_send(buzzer_mailbox, RAPAID);
//    stateTop=TARGET_IDENTIFY;
//    terify_flag=1;
//    ackFlag=0;
//    identifyFlag=0;
//#endif
//  }
//}
//��ʱ���ĳ�ʱ����
void button_entry(void *parameter)
{

    //���水��״̬
    key1_last_status = key1_status;
    key2_last_status = key2_status;
    key3_last_status = key3_status;
    key4_last_status = key4_status;
    key5_last_status = key5_status;
    key6_last_status = key6_status;
//    key7_last_status = key7_status;
    
    //��ȡ��ǰ����״̬
    key1_status = gpio_get(KEY_1);
    key2_status = gpio_get(KEY_2);
    key3_status = gpio_get(KEY_3);
    key4_status = gpio_get(KEY_4);
    key5_status = gpio_get(KEY_5);
    key6_status = gpio_get(KEY_6);
//    key7_status = gpio_get(KEY_7);
    
    //��⵽��������֮�󲢷ſ� �������أ��ͷ�һ���ź���
    if(key1_status && !key1_last_status)    
    {
        rt_sem_release(key1_sem);
    }
    if(key2_status && !key2_last_status)    
    {
        rt_sem_release(key2_sem);
    }
    if(key3_status && !key3_last_status)    
    {
        rt_sem_release(key3_sem);
    }
    if(key4_status && !key4_last_status)    
    {
        rt_sem_release(key4_sem);
    }  
    if(key5_status && !key5_last_status)    
    {
        rt_sem_release(key5_sem);
    }
    if(key6_status && !key6_last_status)    
    {
        rt_sem_release(key6_sem);
    } 
//    if(key7_status && !key7_last_status)    
//    {
//        rt_sem_release(key7_sem);
//    }
}

int button_init(void)
{
    //���ڰ����ж�ͨ����ʱ������������ϵͳ��ʵʱ�ԣ����Ի��ǲ��ö�ʱ��
    rt_timer_t timerForKey;
    //���ؽ���
    rt_thread_t key1_tid;
    rt_thread_t key2_tid;
    rt_thread_t key3_tid;
    rt_thread_t key4_tid;
    rt_thread_t key5_tid;
    rt_thread_t key6_tid;
//    rt_thread_t key7_tid;
    
    gpio_init(KEY_1, GPI, GPIO_LOW, GPIO_KEY_CONFIG);			// ��ʼ��ΪGPIO�������� Ĭ�������ߵ�ƽ
    gpio_init(KEY_2, GPI, GPIO_LOW, GPIO_KEY_CONFIG);
    gpio_init(KEY_3, GPI, GPIO_LOW, GPIO_KEY_CONFIG);
    gpio_init(KEY_4, GPI, GPIO_LOW, GPIO_KEY_CONFIG);
    gpio_init(KEY_5, GPI, GPIO_LOW, GPIO_KEY_CONFIG);
    gpio_init(KEY_6, GPI, GPIO_LOW, GPIO_KEY_CONFIG);
//    gpio_init(KEY_7, GPI, GPIO_HIGH, GPIO_PIN_CONFIG);
    
    key1_sem = rt_sem_create("key1_sem", 0, RT_IPC_FLAG_PRIO);		//�����������ź��������������¾��ͷ��ź���������ִ���̣߳�����Ҫʹ�ð����ĵط���ȡ�ź�������
    key2_sem = rt_sem_create("key2_sem", 0, RT_IPC_FLAG_PRIO);  
    key3_sem = rt_sem_create("key3_sem", 0, RT_IPC_FLAG_PRIO);  
    key4_sem = rt_sem_create("key4_sem", 0, RT_IPC_FLAG_PRIO);
    key5_sem = rt_sem_create("key5_sem", 0, RT_IPC_FLAG_PRIO);  
    key6_sem = rt_sem_create("key6_sem", 0, RT_IPC_FLAG_PRIO); 
//    key7_sem = rt_sem_create("key7_sem", 0, RT_IPC_FLAG_PRIO); 
    
    key1_tid = rt_thread_create("key1_tid", key1_entry, RT_NULL, 1024, 5, 5);  // ���������Ľ��̣���ȡ�ź�����ִ�У�Ȼ���ͷ��ź���
    key2_tid = rt_thread_create("key2_tid", key2_entry, RT_NULL, 1024, 5, 5);  //���������ȼ��Ÿߣ���Ϊ���ٰ�����
    key3_tid = rt_thread_create("key3_tid", key3_entry, RT_NULL, 1024, 5, 5);
    key4_tid = rt_thread_create("key4_tid", key4_entry, RT_NULL, 1024, 5, 5);
    key5_tid = rt_thread_create("key5_tid", key5_entry, RT_NULL, 1024, 5, 5);
    key6_tid = rt_thread_create("key6_tid", key6_entry, RT_NULL, 1024, 5, 5);
//    key7_tid = rt_thread_create("key7_tid", key7_entry, RT_NULL, 1024, 5, 5);
    
    //��������
    if(RT_NULL != key1_tid)
    {
        rt_thread_startup(key1_tid);
    }
    else													// �̴߳���ʧ��
    {
        rt_kprintf("key1_tid thread create ERROR.\n");
        return -1;
    }
    if(RT_NULL != key2_tid)
    {
        rt_thread_startup(key2_tid);
    }
    else													// �̴߳���ʧ��
    {
        rt_kprintf("key2_tid thread create ERROR.\n");
        return -1;
    }
    if(RT_NULL != key3_tid)
    {
        rt_thread_startup(key3_tid);
    }
    else													// �̴߳���ʧ��
    {
        rt_kprintf("key3_tid thread create ERROR.\n");
        return -1;
    }
    if(RT_NULL != key4_tid)
    {
        rt_thread_startup(key4_tid);
    }
    else													// �̴߳���ʧ��
    {
        rt_kprintf("key4_tid thread create ERROR.\n");
        return -1;
    }
    if(RT_NULL != key5_tid)
    {
        rt_thread_startup(key5_tid);
    }
    else													// �̴߳���ʧ��
    {
        rt_kprintf("key5_tid thread create ERROR.\n");
        return -1;
    }
    if(RT_NULL != key6_tid)
    {
        rt_thread_startup(key6_tid);
    }
    else													// �̴߳���ʧ��
    {
        rt_kprintf("key6_tid thread create ERROR.\n");
        return -1;
    }
//    if(RT_NULL != key7_tid)
//    {
//        rt_thread_startup(key7_tid);
//    }
//    else													// �̴߳���ʧ��
//    {
//        rt_kprintf("key7_tid thread create ERROR.\n");
//        return -1;
//    }
    
    
    timerForKey = rt_timer_create("button", button_entry, RT_NULL, 50, RT_TIMER_FLAG_PERIODIC);

    if(RT_NULL != timerForKey) 
    {
        rt_timer_start(timerForKey);
    }
    else													// ʱ�Ӵ���ʧ��
    {
        rt_kprintf("timerForKey create ERROR.\n");
        return -1;
    }
    return 0;
}




