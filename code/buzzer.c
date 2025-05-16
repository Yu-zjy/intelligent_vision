#include "buzzer.h"


rt_mailbox_t buzzer_mailbox;


void buzzer_entry(void *parameter)
{
    rt_ubase_t mb_data;
    
    gpio_set_level(BUZZER_PIN, 1);    //�򿪷�����
    rt_thread_mdelay(100);  //��ʱ
    gpio_set_level(BUZZER_PIN, 0);    //�رշ�����
    while(1)
    {
        //�����������ݣ����û������������ȴ����ͷ�CPU����Ȩ
        rt_mb_recv(buzzer_mailbox, &mb_data, RT_WAITING_FOREVER);

        gpio_set_level(BUZZER_PIN, 1);    //�򿪷�����
        rt_thread_mdelay(mb_data);  //��ʱ
        gpio_set_level(BUZZER_PIN, 0);    //�رշ�����
    }
}





int buzzer_init(void)
{
    rt_thread_t tid;
    
    //��ʼ����������ʹ�õ�GPIO
    gpio_init(BUZZER_PIN, GPO, 0, GPI_PULL_UP);			// ��ʼ��ΪGPIO�������� Ĭ�������ߵ�ƽ
    
    //��������
    buzzer_mailbox = rt_mb_create("buzzer", 5, RT_IPC_FLAG_PRIO);
    
    //�������������߳� ���ȼ�12
    tid = rt_thread_create("buzzer", buzzer_entry, RT_NULL, 256, 5, 2);
    
    //�����߳�
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
    else													// �̴߳���ʧ��
    {
        rt_kprintf("buzzer thread create ERROR.\n");
        return -1;
    }
    return 0;
}
