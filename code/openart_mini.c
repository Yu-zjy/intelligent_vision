#include "openart_mini.h"
uint8 flag=0;
//储存相关
uint8 uart_derection=0;//1left,2right
//left
uint8 cmr_buf[7]={0x00,0x00,0xff,0xff,0xff,0xff,0xff};//头帧ff,aa,尾帧80,7f,1f
uint8 uart_count=1;
uint8 type=0;
uint8 data1,data2,data3,data4;
//right
uint8 cmr_buf2[7]={0x00,0x00,0xff,0xff,0xff,0xff,0xff};//头帧ff,aa,尾帧80,7f,1f
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
//串口中断相关
//left
uint8               openart_rx_buffer_l;
lpuart_transfer_t   openart_receivexfer_l;
lpuart_handle_t     openart_g_lpuartHandle_l;
//right
uint8               openart_rx_buffer_r;
lpuart_transfer_t   openart_receivexfer_r;
lpuart_handle_t     openart_g_lpuartHandle_r;

//标志位
uint8 ackFlag=0;
uint8 identifyFlag=0;
int16_t x_error;
uint16_t h_error;
//坐标识别相关
uint8 numOfCoordinate = 0;
uint8 letter_num_Find=0;
extern uint8 IsCon_get_l,IsCon_get_r;
extern float SetX,SetY,angleSet;
extern findline_TypeDef Findline;
extern CARSTATUS_enum stateTop;

//信号量
rt_sem_t openartl_sem;//一帧数据接收完毕的信号量
rt_sem_t openartr_sem;
    // 方式2：使用联合体处理字节序
     union {
         float f;
         uint8_t bytes[4];
     } converter;

float received_float;

#define IS_ELECTRONIC(category) (category <= 0x05 || (category >= 0x0D && category <= 0x0F))
#define IS_TOOL(category) (category >= 0x07 && category <= 0x0C)
#define IS_NUM(category) (category >= 0x10 && category <= 0x73)  // 数字编码范围
#define PUSH_LEFT  1
#define PUSH_RIGHT 2
typedef enum {
    RECORD_ELECTRONIC,
    RECORD_TOOL,
    RECORD_NUMBER
} RecordType;

typedef struct {
    RecordType type;
    uint8_t category;   // 用于电子外设/常用工具
    uint8_t number;     // 用于手写数字（实际值）
} RecordItem;

RecordItem records[32];
uint8_t record_count = 0;
float backup_X = 0, backup_Y = 0;
uint8_t approachBoxState = 0;    // 0-未激活 1-正在接近 2-调整姿态
//线程入口
void openartl_entry(void *parameter)//通信协议栈
{
  while(1)
  {    
    rt_sem_take(openartl_sem, RT_WAITING_FOREVER);
    ackFlag=1;
    
    switch(type)
    {
    case 0x01://握手
      if(data1==0x81&&data2==0x80&&data3==0x7f&&data4==0x1f) 
      {
        IsCon_get_l=1;
      }
      break;
    case 0x02://直道物品识别
      if(data2==0x80&&data3==0x7f&&data4==0x1f)
      {
        identifyFlag=1;
        mainCategory=data1;
      }
      break;
    case 0x03://数字识别
      if(data2==0x80&&data3==0x7f&&data4==0x1f)
      {
        identifyFlag=1;
        mainCategory=data1;
      }
      break;
    case 0x04://字母识别
      if(data2==0x80&&data3==0x7f&&data4==0x1f)
      {
        identifyFlag=1;
        mainCategory=data1;
      }
       break;
      
    case 0x05://元素物品识别
      if(data2==0x80&&data3==0x7f&&data4==0x1f)
      {
          if(stateTop == ADJUST_POSE)
          {
                                mainCategory = data1;                  
                    // 统一分类逻辑
                    if (IS_NUM(mainCategory)) {
                        // 数字处理
                        uint8_t number = mainCategory - 0x10;  // 转换为实际数字
                        records[record_count].type = RECORD_NUMBER;
                        records[record_count].number = number;
                        record_count++;
                        // 奇偶判断
                        uart_derection = (number % 2 == 1) ? 1 : 2;
                    } 
                    else if (IS_ELECTRONIC(mainCategory)) {
                        // 电子外设
                        records[record_count].type = RECORD_ELECTRONIC;
                        records[record_count].category = mainCategory;
                        record_count++;
                        uart_derection = 1;  // 推左
                    } 
                    else if (IS_TOOL(mainCategory)) {
                        // 常用工具
                        records[record_count].type = RECORD_TOOL;
                        records[record_count].category = mainCategory;
                        record_count++;
                        uart_derection = 2;  // 推右
                    }
            stateTop = (uart_derection % 2) ? PUSH_LEFT : PUSH_RIGHT;  
            terify_flag=1;
          }
            
        
        
      }
      break;

    case 0x06://直道物品定位
      if(data3==0x7f&&data4==0x1f)
      {
        if((data1 !=0xff || data2 !=0xff))
        {
        //仅在巡线状态且未处于其他任务时触发
          if(stateTop == FINDLINE && approachBoxState == 0)
          {
           // 备份当前位置
//           backup_X = encoder.X;
//           backup_Y = encoder.Y;
//           x_error = (int8_t)data5;
//           float dx_local = x_error * 0.0001 * 80; // 缩放系数需校准
//           float dy_local = 1.0f; // 向前移动1米
//             // 设置目标位置
//           positionPIDFlag=1;
//           SetX = backup_X + dx_local;
//           SetY = backup_Y + dy_local;
//          // 切换状态
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
    case 0x07://元素物品定位
      if(data3==0x7f&&data4==0x1f)
      {
        if((data1 !=0xff || data2 !=0xff))
        {
        //惯性坐标系下的位置校正，系数是在保证不振荡的前提下随便取的。倒车，固坐标系运动翻转
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
    case 0x08://字母数字定位
      if(data3==0x7f&&data4==0x1f)
      {
        if((data1 !=0xff || data2 !=0xff))
        {
          if(uart_derection==0) uart_derection=1;
        //惯性坐标系下的位置校正，系数是在保证不振荡的前提下随便取的。倒车，固坐标系运动翻转
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
    default://定位
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
    uart_rx_interrupt(OPENMINIUART_L, 1);//开启中断   
  }
}

void openartr_entry(void *parameter)//通信协议栈
{
  while(1)
  {    
    rt_sem_take(openartr_sem, RT_WAITING_FOREVER);
    ackFlag=1;
    
    switch(type2)
    {
    case 0x01://握手
      if(data5==0x81&&data6==0x80&&data7==0x7f&&data8==0x1f) 
      {
        IsCon_get_r=1;
      }
      break;
    case 0x02://直道物品识别
      if(data6==0x80&&data7==0x7f&&data8==0x1f)
      {
        identifyFlag=1;
        mainCategory=data5;
      }
      break;
    case 0x03://数字识别
      if(data6==0x80&&data7==0x7f&&data8==0x1f)
      {
        identifyFlag=1;
        mainCategory=data5;
      }
      break;
    case 0x04://字母识别
      if(data6==0x80&&data7==0x7f&&data8==0x1f)
      {
        identifyFlag=1;
        mainCategory=data5;
      }
       break;
    case 0x05://元素物品识别
      if(data6==0x80&&data7==0x7f&&data8==0x1f)
      {
        identifyFlag=1;
        mainCategory=data5;
      }
      break;
 //右摄像头识别红箱子在此修改
//      case 0x06://直道物品定位
//        if(data5!=0xff||data6!=0xff){
//           int8 x_error=(int8_t)data5;
//           int8 y_error=(int8_t)data6;
//           //设置目标位置
//           positionPIDFlag=1;
//           SetX = (float)(encoder.X + (float)x_error);
//           SetY = (float)(encoder.Y + (float)y_error);
//           angleSet=0;
//        }
//        else{
//        terify_flag=1;
//        stateTop = ADJUST_POSE;
//        approachBoxState = 2;
//        timeTest_5ms = 0; // 重置超时计时器
//        }
    case 0x06://直道物品定位
      if(!zero_flag)
      {
          if(data5!=0x00||data6!=0x00||data7!=0x00||data8!=0x00)
          {
        //仅在巡线状态且未处于其他任务时触发
          if(stateTop == FINDLINE && approachBoxState == 0)
          {
           // 备份当前位置
//           backup_X = encoder.X;
//           backup_Y = encoder.Y;
//           x_error= (int8_t)data5;
//           h_error=(int8_t)data6;
//           float dx_local = x_error * 0.0001 * 200; // 缩放系数需校准
//           float dy_local = 1.2f; // 向前移动1米
             // 设置目标位置

//           SetX = backup_X + dx_local;
//           SetY = backup_Y + dy_local;
           //angleSet=0;
          // 切换状态
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
    case 0x07://元素物品定位
      if(data7==0x7f&&data8==0x1f)
      {
        if((data5 !=0xff || data6 !=0xff))
        {
        //惯性坐标系下的位置校正，系数是在保证不振荡的前提下随便取的。倒车，固坐标系运动翻转
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
    case 0x08://字母数字定位
      if(data7==0x7f&&data8==0x1f)
      {
        if((data5 !=0xff || data6 !=0xff))
        {
          if(uart_derection==0) uart_derection=2;
        //惯性坐标系下的位置校正，系数是在保证不振荡的前提下随便取的。倒车，固坐标系运动翻转
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
    default://定位
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
    uart_rx_interrupt(OPENMINIUART_R, 1);//开启中断   
  }
}

void openart_uart1_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{   

    rt_interrupt_enter();
    uint8 uart_data1;
    flag=1;
    if(kStatus_LPUART_RxIdle == status)
    {
        uart_data1 =openart_rx_buffer_l;    //串口收到数据后会自动进入到这里，然后读取openart_rx_buffer变量即可读取串口收到的数据
     flag = 2;
    }
     
    //帧头1判断
    if(cmr_buf[0]!=0xff)
    {
      cmr_buf[0]=uart_data1;
      flag = 3;

    }
    else
    {
      //帧头2判断
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
        //帧头正确则接收数据
        else
        {
          cmr_buf[++uart_count]=uart_data1;
          if(uart_count==6)
          {          
            //处理数据
            type=cmr_buf[2];
            data1=cmr_buf[3];
            data2=cmr_buf[4];
            data3=cmr_buf[5];
            data4=cmr_buf[6];

            rt_sem_release(openartl_sem);
            uart_rx_interrupt(OPENMINIUART_L, 0);//关闭中断    
            cmr_buf[0]=0x00;
            cmr_buf[1]=0x00;
            cmr_buf[2]=0x00;
            cmr_buf[3]=0x00;
            cmr_buf[4]=0xff;
            cmr_buf[5]=0xff;
            cmr_buf[6]=0xff;
            uart_count=1;
            
            //接受中断交由顶层逻辑控制
          }
        }
    }
    
    handle->rxDataSize = openart_receivexfer_l.dataSize;  //还原缓冲区长度
    handle->rxData = openart_receivexfer_l.data;          //还原缓冲区地址
    
    rt_interrupt_leave();
}

void openart_uart4_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
    rt_interrupt_enter();
    uint8 uart_data2;
    
    if(kStatus_LPUART_RxIdle == status)
    {
        uart_data2 =openart_rx_buffer_r;    //串口收到数据后会自动进入到这里，然后读取openart_rx_buffer变量即可读取串口收到的数据
    }
     
    //帧头1判断
    if(cmr_buf2[0]!=0xff)
    {
      cmr_buf2[0]=uart_data2;
    }
    else
    {
      //帧头2判断
        if(cmr_buf2[1]!=0xAA)
        {
          cmr_buf2[1]=uart_data2;
          if(cmr_buf2[1]!=0xAA)
          {
            cmr_buf2[0]=uart_data2;
            cmr_buf2[1]=0;
          }
        }
        //帧头正确则接收数据
        else
        {
          cmr_buf2[++uart_count2]=uart_data2;
          if(uart_count2==6)
          {          
            //处理数据
            type2=cmr_buf2[2];
            data5=cmr_buf2[3];
            data6=cmr_buf2[4];
            data7=cmr_buf2[5];
            data8=cmr_buf2[6];
                 for (int i = 0; i < 4; i++) {
         converter.bytes[i] = cmr_buf2[3]; // 小端：buffer[0]是低字节
     }
     received_float = converter.f;

            rt_sem_release(openartr_sem);
            uart_rx_interrupt(OPENMINIUART_R, 0);//关闭中断    
            cmr_buf2[0]=0x00;
            cmr_buf2[1]=0x00;
            cmr_buf2[2]=0x00;
            cmr_buf2[3]=0x00;
            cmr_buf2[4]=0xff;
            cmr_buf2[5]=0xff;
            cmr_buf2[6]=0xff;
            uart_count2=1;
            
            //接受中断交由顶层逻辑控制
          }
        }
    }
    
    handle->rxDataSize = openart_receivexfer_r.dataSize;  //还原缓冲区长度
    handle->rxData = openart_receivexfer_r.data;          //还原缓冲区地址
    
    rt_interrupt_leave();
}

void openart_mini(void)
{
    rt_thread_t openartl_tid,openartr_tid;//通信协议栈
    uart_init(OPENMINIUART_L, 115200, UART1_TX_B12, UART1_RX_B13);
    uart_init(OPENMINIUART_R, 115200, UART4_TX_C16, UART4_RX_C17);
    
    //配置串口接收的缓冲区及缓冲区长度
    openart_receivexfer_l.dataSize = 1;
    openart_receivexfer_l.data = &openart_rx_buffer_l;
    
    openart_receivexfer_r.dataSize = 1;
    openart_receivexfer_r.data = &openart_rx_buffer_r;

    //设置中断函数及其参数
    uart_set_handle(OPENMINIUART_L, &openart_g_lpuartHandle_l, openart_uart1_callback, NULL, 0, openart_receivexfer_l.data, 1);
    uart_set_handle(OPENMINIUART_R, &openart_g_lpuartHandle_r, openart_uart4_callback, NULL, 0, openart_receivexfer_r.data, 1);
    
    NVIC_SetPriority(LPUART1_IRQn,0);         //设置串口中断优先级 范围0-15 越小优先级越高
    uart_rx_interrupt(OPENMINIUART_L,1); //开启中断
    
    NVIC_SetPriority(LPUART4_IRQn,1);         //设置串口中断优先级 范围0-15 越小优先级越高
    uart_rx_interrupt(OPENMINIUART_R,1); //开启中断

    

    openartl_sem = rt_sem_create("openartl_sem", 0, RT_IPC_FLAG_PRIO);
    openartr_sem = rt_sem_create("openartr_sem", 0, RT_IPC_FLAG_PRIO);
    
    openartl_tid = rt_thread_create("openartl_tid", openartl_entry, RT_NULL, 2048, 0, 5);
    if(RT_NULL != openartl_tid)
    {
        rt_thread_startup(openartl_tid);
    }
    else													// 线程创建失败
    {
        rt_kprintf("openartl_tid thread create ERROR.\n");
        return ;
    }
    
    openartr_tid = rt_thread_create("openartr_tid", openartr_entry, RT_NULL, 2048, 1, 5);
    if(RT_NULL != openartr_tid)
    {
        rt_thread_startup(openartr_tid);
    }
    else													// 线程创建失败
    {
        rt_kprintf("openartr_tid thread create ERROR.\n");
        return ;
    }

}