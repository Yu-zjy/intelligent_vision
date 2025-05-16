#include "openart_mini.h"
uint8 flag=0;
//�������
uint8 uart_derection=0;//1left,2right
//left
uint8 cmr_buf[7]={0x00,0x00,0xff,0xff,0xff,0xff,0xff};//ͷ֡ff,aa,β֡80,7f,1f
uint8 uart_count=1;
uint8 type=0;
uint8 data1,data2,data3,data4;
//right
uint8 cmr_buf2[7]={0x00,0x00,0xff,0xff,0xff,0xff,0xff};//ͷ֡ff,aa,β֡80,7f,1f
uint8 uart_count2=1;
uint8 type2=0;
uint8 data5=0xff;
uint8 data6=0xff;
uint8 data7=0xff;
uint8 data8=0xff;
uint8 mainCategory=0,minorCategory=0;
uint8 imageProcessFinish_flag=0;
uint8 imageProcess_found=0;
uint8 zero_flag=0;
//�����ж����
//left
uint8               openart_rx_buffer_l;
lpuart_transfer_t   openart_receivexfer_l;
lpuart_handle_t     openart_g_lpuartHandle_l;
//right
uint8               openart_rx_buffer_r;
lpuart_transfer_t   openart_receivexfer_r;
lpuart_handle_t     openart_g_lpuartHandle_r;

//��־λ
uint8 ackFlag=0;
uint8 identifyFlag=0;
int16_t x_error;
uint16_t h_error;
//����ʶ�����
uint8 numOfCoordinate = 0;
uint8 letter_num_Find=0;
extern uint8 IsCon_get_l,IsCon_get_r;
extern float SetX,SetY,angleSet;
extern findline_TypeDef Findline;
extern CARSTATUS_enum stateTop;

//�ź���
rt_sem_t openartl_sem;//һ֡���ݽ�����ϵ��ź���
rt_sem_t openartr_sem;
    // ��ʽ2��ʹ�������崦���ֽ���
     union {
         float f;
         uint8_t bytes[4];
     } converter;

float received_float;

#define IS_ELECTRONIC(category) (category <= 0x05 || (category >= 0x0D && category <= 0x0F))
#define IS_TOOL(category) (category >= 0x07 && category <= 0x0C)
#define IS_NUM(category) (category >= 0x10 && category <= 0x73)  // ���ֱ��뷶Χ
#define PUSH_LEFT  1
#define PUSH_RIGHT 2
typedef enum {
    RECORD_ELECTRONIC,
    RECORD_TOOL,
    RECORD_NUMBER
} RecordType;

typedef struct {
    RecordType type;
    uint8_t category;   // ���ڵ�������/���ù���
    uint8_t number;     // ������д���֣�ʵ��ֵ��
} RecordItem;

RecordItem records[32];
uint8_t record_count = 0;
float backup_X = 0, backup_Y = 0;
uint8_t approachBoxState = 0;    // 0-δ���� 1-���ڽӽ� 2-������̬
//�߳����
void openartl_entry(void *parameter)//ͨ��Э��ջ
{
  while(1)
  {    
    rt_sem_take(openartl_sem, RT_WAITING_FOREVER);
    ackFlag=1;
    
    switch(type)
    {
    case 0x01://����
      if(data1==0x81&&data2==0x80&&data3==0x7f&&data4==0x1f) 
      {
        IsCon_get_l=1;
      }
      break;
    case 0x02://ֱ����Ʒʶ��
      if(data2==0x80&&data3==0x7f&&data4==0x1f)
      {
        identifyFlag=1;
        mainCategory=data1;
      }
      break;
    case 0x03://����ʶ��
      if(data2==0x80&&data3==0x7f&&data4==0x1f)
      {
        identifyFlag=1;
        mainCategory=data1;
      }
      break;
    case 0x04://��ĸʶ��
      if(data2==0x80&&data3==0x7f&&data4==0x1f)
      {
        identifyFlag=1;
        mainCategory=data1;
      }
       break;
      
    case 0x05://Ԫ����Ʒʶ��
      if(data2==0x80&&data3==0x7f&&data4==0x1f)
      {
          if(stateTop == ADJUST_POSE)
          {
                                mainCategory = data1;                  
                    // ͳһ�����߼�
                    if (IS_NUM(mainCategory)) {
                        // ���ִ���
                        uint8_t number = mainCategory - 0x10;  // ת��Ϊʵ������
                        records[record_count].type = RECORD_NUMBER;
                        records[record_count].number = number;
                        record_count++;
                        // ��ż�ж�
                        uart_derection = (number % 2 == 1) ? 1 : 2;
                    } 
                    else if (IS_ELECTRONIC(mainCategory)) {
                        // ��������
                        records[record_count].type = RECORD_ELECTRONIC;
                        records[record_count].category = mainCategory;
                        record_count++;
                        uart_derection = 1;  // ����
                    } 
                    else if (IS_TOOL(mainCategory)) {
                        // ���ù���
                        records[record_count].type = RECORD_TOOL;
                        records[record_count].category = mainCategory;
                        record_count++;
                        uart_derection = 2;  // ����
                    }
            stateTop = (uart_derection % 2) ? PUSH_LEFT : PUSH_RIGHT;  
            terify_flag=1;
          }
            
        
        
      }
      break;

    case 0x06://ֱ����Ʒ��λ
      if(data3==0x7f&&data4==0x1f)
      {
        if((data1 !=0xff || data2 !=0xff))
        {
        //����Ѳ��״̬��δ������������ʱ����
          if(stateTop == FINDLINE && approachBoxState == 0)
          {
           // ���ݵ�ǰλ��
//           backup_X = encoder.X;
//           backup_Y = encoder.Y;
//           x_error = (int8_t)data5;
//           float dx_local = x_error * 0.0001 * 80; // ����ϵ����У׼
//           float dy_local = 1.0f; // ��ǰ�ƶ�1��
//             // ����Ŀ��λ��
//           positionPIDFlag=1;
//           SetX = backup_X + dx_local;
//           SetY = backup_Y + dy_local;
//          // �л�״̬
           stateTop = APPROACH_BOX;
           approachBoxState = 1;
          }

        }
        else 
        {
          imageProcessFinish_flag=1;
          imageProcess_found = 0;
        }
      }
      break;
    case 0x07://Ԫ����Ʒ��λ
      if(data3==0x7f&&data4==0x1f)
      {
        if((data1 !=0xff || data2 !=0xff))
        {
        //��������ϵ�µ�λ��У����ϵ�����ڱ�֤���񵴵�ǰ�������ȡ�ġ�������������ϵ�˶���ת
          if(letter_num_Find)
          {
            letter_num_Find=0;
          }
          if(abs(MTXL_EXP_l-data1)<6 && abs(data2-MTYL_EXP_l)<10)
          {
            imageProcessFinish_flag=1;
            imageProcess_found = 1;
          }
          else
          {
            float dyb=(data1-MTXL_EXP_l)*0.0001*10;
            float dxb=-(MTYL_EXP_l-data2)*0.0001*20;
            SetX = encoder.X + dyb*sin(PI*gyro.TurnAngle_Integral/180) + dxb*cos(PI*gyro.TurnAngle_Integral/180);
            SetY = encoder.Y + dyb*cos(PI*gyro.TurnAngle_Integral/180) - dxb*sin(PI*gyro.TurnAngle_Integral/180);
            imageProcessFinish_flag=0;
            imageProcess_found = 1;
          }
        }
        else
        {
          imageProcessFinish_flag=1;
          imageProcess_found = 0;
        }
      }
      break;
    case 0x08://��ĸ���ֶ�λ
      if(data3==0x7f&&data4==0x1f)
      {
        if((data1 !=0xff || data2 !=0xff))
        {
          if(uart_derection==0) uart_derection=1;
        //��������ϵ�µ�λ��У����ϵ�����ڱ�֤���񵴵�ǰ�������ȡ�ġ�������������ϵ�˶���ת
          if(letter_num_Find)
          {
            letter_num_Find=0;
            if(abs(LETER_NUM_MTXL_EXP_l-data1)<15 && abs(data2-LETER_NUM_MTYL_EXP_l)<20)
            {
              imageProcessFinish_flag=1;
              imageProcess_found = 1;
            }
            else
            {
              float dyb=(data1-LETER_NUM_MTXL_EXP_l)*0.0001*10;
              float dxb=-(LETER_NUM_MTYL_EXP_l-data2)*0.0001*20;
              SetX = encoder.X + dyb*sin(PI*gyro.TurnAngle_Integral/180) + dxb*cos(PI*gyro.TurnAngle_Integral/180);
              SetY = encoder.Y + dyb*cos(PI*gyro.TurnAngle_Integral/180) - dxb*sin(PI*gyro.TurnAngle_Integral/180);
              imageProcessFinish_flag=0;
              imageProcess_found = 1;
            }
          }
        }
        else
        {
          imageProcessFinish_flag=1;
          imageProcess_found = 0;
        }
      }
      break;
    default://��λ
      break;
    }
    cmr_buf[0]=0x01;
    cmr_buf[1]=0x00;
    cmr_buf[2]=0x00;
    cmr_buf[3]=0x00;
    cmr_buf[4]=0xff;
    cmr_buf[5]=0xff;
    cmr_buf[6]=0xff;
    uart_count=1;
    uart_rx_interrupt(OPENMINIUART_L, 1);//�����ж�   
  }
}

void openartr_entry(void *parameter)//ͨ��Э��ջ
{
  while(1)
  {    
    rt_sem_take(openartr_sem, RT_WAITING_FOREVER);
    ackFlag=1;
    
    switch(type2)
    {
    case 0x01://����
      if(data5==0x81&&data6==0x80&&data7==0x7f&&data8==0x1f) 
      {
        IsCon_get_r=1;
      }
      break;
    case 0x02://ֱ����Ʒʶ��
      if(data6==0x80&&data7==0x7f&&data8==0x1f)
      {
        identifyFlag=1;
        mainCategory=data5;
      }
      break;
    case 0x03://����ʶ��
      if(data6==0x80&&data7==0x7f&&data8==0x1f)
      {
        identifyFlag=1;
        mainCategory=data5;
      }
      break;
    case 0x04://��ĸʶ��
      if(data6==0x80&&data7==0x7f&&data8==0x1f)
      {
        identifyFlag=1;
        mainCategory=data5;
      }
       break;
    case 0x05://Ԫ����Ʒʶ��
      if(data6==0x80&&data7==0x7f&&data8==0x1f)
      {
        identifyFlag=1;
        mainCategory=data5;
      }
      break;
 //������ͷʶ��������ڴ��޸�
//      case 0x06://ֱ����Ʒ��λ
//        if(data5!=0xff||data6!=0xff){
//           int8 x_error=(int8_t)data5;
//           int8 y_error=(int8_t)data6;
//           //����Ŀ��λ��
//           positionPIDFlag=1;
//           SetX = (float)(encoder.X + (float)x_error);
//           SetY = (float)(encoder.Y + (float)y_error);
//           angleSet=0;
//        }
//        else{
//        terify_flag=1;
//        stateTop = ADJUST_POSE;
//        approachBoxState = 2;
//        timeTest_5ms = 0; // ���ó�ʱ��ʱ��
//        }
    case 0x06://ֱ����Ʒ��λ
      if(!zero_flag)
      {
          if(data5!=0x00||data6!=0x00||data7!=0x00||data8!=0x00)
          {
        //����Ѳ��״̬��δ������������ʱ����
          if(stateTop == FINDLINE && approachBoxState == 0)
          {
           // ���ݵ�ǰλ��
//           backup_X = encoder.X;
//           backup_Y = encoder.Y;
//           x_error= (int8_t)data5;
//           h_error=(int8_t)data6;
//           float dx_local = x_error * 0.0001 * 200; // ����ϵ����У׼
//           float dy_local = 1.2f; // ��ǰ�ƶ�1��
             // ����Ŀ��λ��

//           SetX = backup_X + dx_local;
//           SetY = backup_Y + dy_local;
           //angleSet=0;
          // �л�״̬
            positionPIDFlag=2;
           stateTop = APPROACH_BOX;
           approachBoxState = 1;
          }
           x_error=(int16_t)((data5 << 8)+data6);
           h_error=(data7 << 8)+data8;
      }
      else{
           x_error=0;
           h_error=0;
           positionPIDFlag=1;
           zero_flag=1;
      }
      }
      
      break;
    case 0x07://Ԫ����Ʒ��λ
      if(data7==0x7f&&data8==0x1f)
      {
        if((data5 !=0xff || data6 !=0xff))
        {
        //��������ϵ�µ�λ��У����ϵ�����ڱ�֤���񵴵�ǰ�������ȡ�ġ�������������ϵ�˶���ת
          if(letter_num_Find)
          {
            letter_num_Find=0;
          }
          if(abs(MTXL_EXP_r-data5)<6 && abs(data6-MTYL_EXP_r)<10)
          {
            imageProcessFinish_flag=1;
            imageProcess_found = 1;
          }
          else
          {
            float dyb=-(data5-MTXL_EXP_r)*0.0001*10;
            float dxb=(MTYL_EXP_r-data6)*0.0001*20;
            SetX = encoder.X + dyb*sin(PI*gyro.TurnAngle_Integral/180) + dxb*cos(PI*gyro.TurnAngle_Integral/180);
            SetY = encoder.Y + dyb*cos(PI*gyro.TurnAngle_Integral/180) - dxb*sin(PI*gyro.TurnAngle_Integral/180);
            imageProcessFinish_flag=0;
            imageProcess_found = 1;
          }
        }
        else
        {
          imageProcessFinish_flag=1;
          imageProcess_found = 0;
        }
      }
      break;
    case 0x08://��ĸ���ֶ�λ
      if(data7==0x7f&&data8==0x1f)
      {
        if((data5 !=0xff || data6 !=0xff))
        {
          if(uart_derection==0) uart_derection=2;
        //��������ϵ�µ�λ��У����ϵ�����ڱ�֤���񵴵�ǰ�������ȡ�ġ�������������ϵ�˶���ת
          if(letter_num_Find)
          {
            letter_num_Find=0;        
            if(abs(LETER_NUM_MTXL_EXP_r-data5)<15 && abs(data6-LETER_NUM_MTYL_EXP_r)<20)
            {
              imageProcessFinish_flag=1;
              imageProcess_found = 1;
            }
            else
            {
              float dyb=-(data5-LETER_NUM_MTXL_EXP_r)*0.0001*10;
              float dxb=(LETER_NUM_MTYL_EXP_r-data6)*0.0001*20;
              SetX = encoder.X + dyb*sin(PI*gyro.TurnAngle_Integral/180) + dxb*cos(PI*gyro.TurnAngle_Integral/180);
              SetY = encoder.Y + dyb*cos(PI*gyro.TurnAngle_Integral/180) - dxb*sin(PI*gyro.TurnAngle_Integral/180);
              imageProcessFinish_flag=0;
              imageProcess_found = 1;
            }
          }
        }
        else
        {
          imageProcessFinish_flag=1;
          imageProcess_found = 0;
        }
      }
      break;
    default://��λ
      break;
    }
    cmr_buf2[0]=0x00;
    cmr_buf2[1]=0x00;
    cmr_buf2[2]=0x00;
    cmr_buf2[3]=0x00;
    cmr_buf2[4]=0xff;
    cmr_buf2[5]=0xff;
    cmr_buf2[6]=0xff;
    uart_count2=1;
    uart_rx_interrupt(OPENMINIUART_R, 1);//�����ж�   
  }
}

void openart_uart1_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{   

    rt_interrupt_enter();
    uint8 uart_data1;
    flag=1;
    if(kStatus_LPUART_RxIdle == status)
    {
        uart_data1 =openart_rx_buffer_l;    //�����յ����ݺ���Զ����뵽���Ȼ���ȡopenart_rx_buffer�������ɶ�ȡ�����յ�������
     flag = 2;
    }
     
    //֡ͷ1�ж�
    if(cmr_buf[0]!=0xff)
    {
      cmr_buf[0]=uart_data1;
      flag = 3;

    }
    else
    {
      //֡ͷ2�ж�
        if(cmr_buf[1]!=0xAA)
        {
          cmr_buf[1]=uart_data1;
          flag = 4;
          if(cmr_buf[1]!=0xAA)
          {
            cmr_buf[0]=uart_data1;
            cmr_buf[1]=0;

          }
        }
        //֡ͷ��ȷ���������
        else
        {
          cmr_buf[++uart_count]=uart_data1;
          if(uart_count==6)
          {          
            //��������
            type=cmr_buf[2];
            data1=cmr_buf[3];
            data2=cmr_buf[4];
            data3=cmr_buf[5];
            data4=cmr_buf[6];

            rt_sem_release(openartl_sem);
            uart_rx_interrupt(OPENMINIUART_L, 0);//�ر��ж�    
            cmr_buf[0]=0x00;
            cmr_buf[1]=0x00;
            cmr_buf[2]=0x00;
            cmr_buf[3]=0x00;
            cmr_buf[4]=0xff;
            cmr_buf[5]=0xff;
            cmr_buf[6]=0xff;
            uart_count=1;
            
            //�����жϽ��ɶ����߼�����
          }
        }
    }
    
    handle->rxDataSize = openart_receivexfer_l.dataSize;  //��ԭ����������
    handle->rxData = openart_receivexfer_l.data;          //��ԭ��������ַ
    
    rt_interrupt_leave();
}

void openart_uart4_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
    rt_interrupt_enter();
    uint8 uart_data2;
    
    if(kStatus_LPUART_RxIdle == status)
    {
        uart_data2 =openart_rx_buffer_r;    //�����յ����ݺ���Զ����뵽���Ȼ���ȡopenart_rx_buffer�������ɶ�ȡ�����յ�������
    }
     
    //֡ͷ1�ж�
    if(cmr_buf2[0]!=0xff)
    {
      cmr_buf2[0]=uart_data2;
    }
    else
    {
      //֡ͷ2�ж�
        if(cmr_buf2[1]!=0xAA)
        {
          cmr_buf2[1]=uart_data2;
          if(cmr_buf2[1]!=0xAA)
          {
            cmr_buf2[0]=uart_data2;
            cmr_buf2[1]=0;
          }
        }
        //֡ͷ��ȷ���������
        else
        {
          cmr_buf2[++uart_count2]=uart_data2;
          if(uart_count2==6)
          {          
            //��������
            type2=cmr_buf2[2];
            data5=cmr_buf2[3];
            data6=cmr_buf2[4];
            data7=cmr_buf2[5];
            data8=cmr_buf2[6];
                 for (int i = 0; i < 4; i++) {
         converter.bytes[i] = cmr_buf2[3]; // С�ˣ�buffer[0]�ǵ��ֽ�
     }
     received_float = converter.f;

            rt_sem_release(openartr_sem);
            uart_rx_interrupt(OPENMINIUART_R, 0);//�ر��ж�    
            cmr_buf2[0]=0x00;
            cmr_buf2[1]=0x00;
            cmr_buf2[2]=0x00;
            cmr_buf2[3]=0x00;
            cmr_buf2[4]=0xff;
            cmr_buf2[5]=0xff;
            cmr_buf2[6]=0xff;
            uart_count2=1;
            
            //�����жϽ��ɶ����߼�����
          }
        }
    }
    
    handle->rxDataSize = openart_receivexfer_r.dataSize;  //��ԭ����������
    handle->rxData = openart_receivexfer_r.data;          //��ԭ��������ַ
    
    rt_interrupt_leave();
}

void openart_mini(void)
{
    rt_thread_t openartl_tid,openartr_tid;//ͨ��Э��ջ
    uart_init(OPENMINIUART_L, 115200, UART1_TX_B12, UART1_RX_B13);
    uart_init(OPENMINIUART_R, 115200, UART4_TX_C16, UART4_RX_C17);
    
    //���ô��ڽ��յĻ�����������������
    openart_receivexfer_l.dataSize = 1;
    openart_receivexfer_l.data = &openart_rx_buffer_l;
    
    openart_receivexfer_r.dataSize = 1;
    openart_receivexfer_r.data = &openart_rx_buffer_r;

    //�����жϺ����������
    uart_set_handle(OPENMINIUART_L, &openart_g_lpuartHandle_l, openart_uart1_callback, NULL, 0, openart_receivexfer_l.data, 1);
    uart_set_handle(OPENMINIUART_R, &openart_g_lpuartHandle_r, openart_uart4_callback, NULL, 0, openart_receivexfer_r.data, 1);
    
    NVIC_SetPriority(LPUART1_IRQn,0);         //���ô����ж����ȼ� ��Χ0-15 ԽС���ȼ�Խ��
    uart_rx_interrupt(OPENMINIUART_L,1); //�����ж�
    
    NVIC_SetPriority(LPUART4_IRQn,1);         //���ô����ж����ȼ� ��Χ0-15 ԽС���ȼ�Խ��
    uart_rx_interrupt(OPENMINIUART_R,1); //�����ж�

    

    openartl_sem = rt_sem_create("openartl_sem", 0, RT_IPC_FLAG_PRIO);
    openartr_sem = rt_sem_create("openartr_sem", 0, RT_IPC_FLAG_PRIO);
    
    openartl_tid = rt_thread_create("openartl_tid", openartl_entry, RT_NULL, 2048, 0, 5);
    if(RT_NULL != openartl_tid)
    {
        rt_thread_startup(openartl_tid);
    }
    else													// �̴߳���ʧ��
    {
        rt_kprintf("openartl_tid thread create ERROR.\n");
        return ;
    }
    
    openartr_tid = rt_thread_create("openartr_tid", openartr_entry, RT_NULL, 2048, 1, 5);
    if(RT_NULL != openartr_tid)
    {
        rt_thread_startup(openartr_tid);
    }
    else													// �̴߳���ʧ��
    {
        rt_kprintf("openartr_tid thread create ERROR.\n");
        return ;
    }

}