/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� RT1064DVL6A ��Դ���һ����
* 
* RT1064DVL6A ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
* 
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
* 
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
* 
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
* 
* �ļ�����          main
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 8.32.4 or MDK 5.33
* ����ƽ̨          RT1064DVL6A
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
extern uint8 road_type;
extern findline_TypeDef Findline;
  int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // ����ɾ��
    	        
    system_delay_ms(500);           //�ȴ��������������ϵ����
    
    pit_self_init();                     //��ʼ��pit����
//  
//  //������ʼ��
  buzzer_init();
  encoder_init();
  motor_init();
  ips200_init(IPS200_TYPE_SPI);
  gyroscopeInit();//�����ǳ�ʼ�� 
  controlInit();
image_init();
mt9v03x_init();
  
////  wireless_init();
   openart_mini();
//  smotor_init();
//  detector_init();
//    
//    //�жϳ�ʼ�� ��������ʼ������Ϊ��û��disable�жϣ���֪��Ϊʲô��
////    wdogInterrupt_init();//���Ź���ʼ����ι��ʱ��鿴��صĺ궨�塣�������⿨��������ι��ʱ�䣩ʱ�����������λ
//    
////
////    rt_mb_send(buzzer_mailbox, 50);
    timer_pit_init();
//    uart_putstr(WIRELESS_UART, "ping...\n");
    gpio_init(B9,GPO,0,GPI_PULL_UP);
    
    Priority_Init();
    
    interrupt_global_enable(0);

    
    while(1)
    {
     gpio_toggle_level(B9);//��ת���ŵ�ƽ
        rt_thread_mdelay(500);	//���߳̽���CPUʹ��Ȩ
        //ips200_displayimage03x(mt9v03x_image_pross3[0], 240, 300);
//        ips200_show_int(10,200,road_type,2);
//        ips200_show_int(10,240,Findline.straightend,3);
//        ips200_show_int(10,280,Findline.endline,3);
        //ips200_displayimage03x(mt9v03x_image_pross3[0], 240, 300);
    }
}

// **************************** �������� ****************************



