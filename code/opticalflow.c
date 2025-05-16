#include "opticalflow.h"

rt_sem_t opticalflow_sem;//һ֡���ݽ�����ϵ��ź���

//�����ж����
uint8               opticalflow_rx_buffer;
lpuart_transfer_t   opticalflow_receivexfer;
lpuart_handle_t     opticalflow_g_lpuartHandle;
//�������
uint8 cmr_buff[9]={0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff};
uint8 uart_count_=1;
int16_t flow_x,flow_y;
uint8 dataXL,dataXH,dataYL,dataYH;
int32 flow_xAccum=0,flow_yAccum=0;
//�����ⲿͨ�ű�־λ
uint8 opticalflowFlag = 0;

void opticalflow_uart8_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
    rt_interrupt_enter();
    
    uint8 uart_data1;
    uint8_t Check_sum = 0;
    
    if(kStatus_LPUART_RxIdle == status)
    {
        uart_data1 =opticalflow_rx_buffer;    //�����յ����ݺ���Զ����뵽���Ȼ���ȡopticalflow_rx_buffer�������ɶ�ȡ�����յ�������
    }
     
    
    //֡ͷ1�ж�
    if(cmr_buff[0]!=0xfe)
    {
      cmr_buff[0]=uart_data1;
    }
    else
    {
      //֡ͷ��ȷ���������
        cmr_buff[uart_count_++]=uart_data1;
        if(uart_count_==9)
        {          
          //��������
          dataXL=cmr_buff[2];
          dataXH=cmr_buff[3];
          dataYL=cmr_buff[4];
          dataYH=cmr_buff[5];
          Check_sum =  (uint8_t)(dataXL+dataXH+dataYL+dataYH);
          if(Check_sum == cmr_buff[6])// check
            rt_sem_release(opticalflow_sem);
//            uart_rx_irq(OPENMINIUART, 0);//�ر��ж�    
          cmr_buff[0]=0x00;
          cmr_buff[1]=0x00;
          cmr_buff[2]=0xff;
          cmr_buff[3]=0xff;
          cmr_buff[4]=0xff;
          cmr_buff[5]=0xff;
          cmr_buff[6]=0xff;
          cmr_buff[7]=0xff;
          cmr_buff[8]=0xff;
          uart_count_=1;
          
          //�����жϽ��ɶ����߼�����
      }
    }
    
    handle->rxDataSize = opticalflow_receivexfer.dataSize;  //��ԭ����������
    handle->rxData = opticalflow_receivexfer.data;          //��ԭ��������ַ
    
    rt_interrupt_leave();

}

void opticalflow_entry(void *parameter)//ͨ��Э��ջ
{
  while(1)
  {
    rt_sem_take(opticalflow_sem, RT_WAITING_FOREVER);
    
    flow_x = dataXL+(dataXH<<8);
    flow_y = dataYL+(dataYH<<8);
    flow_xAccum += flow_x;
    flow_yAccum += flow_y;
    opticalflowFlag = 1;
  }
}

void opticalflow_init(void)
{
    rt_thread_t opticalflow_tid;//ͨ��Э��ջ
    uart_init(OPTICALFLOW_USART, 19200, UART8_TX_D16, UART8_RX_D17);
    
    //���ô��ڽ��յĻ�����������������
    opticalflow_receivexfer.dataSize = 1;
    opticalflow_receivexfer.data = &opticalflow_rx_buffer;
    
    NVIC_SetPriority(LPUART1_IRQn,0);         //���ô����ж����ȼ� ��Χ0-15 ԽС���ȼ�Խ��
    uart_rx_irq(OPTICALFLOW_USART,1); //�����ж�
    //�����жϺ����������
    uart_set_handle(OPTICALFLOW_USART, &opticalflow_g_lpuartHandle, opticalflow_uart8_callback, NULL, 0, opticalflow_receivexfer.data, 1);
    

    opticalflow_sem = rt_sem_create("opticalflow_sem", 0, RT_IPC_FLAG_PRIO);
    
    opticalflow_tid = rt_thread_create("opticalflow_tid", opticalflow_entry, RT_NULL, 512, 0, 2);
    //��������
    if(RT_NULL != opticalflow_tid)
    {
        rt_thread_startup(opticalflow_tid);
    }
    else													// �̴߳���ʧ��
    {
        rt_kprintf("opticalflow_tid thread create ERROR.\n");
        return ;
    }

}
//��ʱ���ĳ�ʱ����
void opticalflow_dis_entry(void *parameter)
{
  lcd_showint32(10,1,flow_yAccum,6);
  lcd_showint32(10,5,flow_xAccum,6);
}
int8 opticalflow_dis_init(void)
{
  rt_timer_t timerForopticalflow;
  timerForopticalflow = rt_timer_create("timerForopticalflow", opticalflow_dis_entry, RT_NULL, 500, RT_TIMER_FLAG_PERIODIC);

  if(RT_NULL != timerForopticalflow) 
  {
      rt_timer_start(timerForopticalflow);
  }
  else													// ʱ�Ӵ���ʧ��
  {
      rt_kprintf("timerForopticalflow create ERROR.\n");
      return -1;
  }
  lcd_clear(WHITE);
  lcd_showstr(0,0,"Y:");
  lcd_showstr(0,4,"X:");
  return 0;
}
