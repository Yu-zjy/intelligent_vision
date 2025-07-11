#include "wireless.h"


rt_sem_t wireless_sem;

uint8 positionSetCopyX[6];
uint8 positionSetCopyY[6];
uint8 wifi_ready=0;

void wirelessTimer_entry(void *parameter);

#define OPEN_WIRELESS           1
#define INCLUDE_BOUNDARY_TYPE   0


#define WIFI_SSID_TEST          "DESKTOP-DHJ5T1N3061"
#define WIFI_PASSWORD_TEST      "917H4852"  // 如果需要连接的WIFI 没有密码则需要将 "12345678" 替换为 NULL




// 边界的点数量远大于图像高度，便于保存回弯的情况
#define BOUNDARY_NUM            (MT9V03X_H * 3 / 2)

// 只有X边界
uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];

// 只有Y边界
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

// X Y边界都是单独指定的
uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];
uint8 y1_boundary[MT9V03X_W], y2_boundary[MT9V03X_W], y3_boundary[MT9V03X_W];

// 图像备份数组，在发送前将图像备份再进行发送，这样可以避免图像出现撕裂的问题
//uint8 image_copy[MT9V03X_H][MT9V03X_W];

void wireless_entry(void *parameter)
{
  while(1)
  {
    rt_sem_take(wireless_sem, RT_WAITING_FOREVER);
        // 在发送前将图像备份再进行发送，这样可以避免图像出现撕裂的问题
#if(OPEN_WIRELESS)
//    memcpy(mt9v03x_image_pross1[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    seekfree_assistant_camera_send();
#endif
//    ips200_displayimage03x(mt9v03x_image_pross3[0], 240, 180);
  }
}



void wireless_init(void)
{
  rt_thread_t wireless_tid;

#if(0 != INCLUDE_BOUNDARY_TYPE)
  int32 i = 0;
#elif(3 == INCLUDE_BOUNDARY_TYPE)
  int32 j = 0;
#endif
  
#if(OPEN_WIRELESS)
  while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
  {
      printf("\r\n connect wifi failed. \r\n");
      system_delay_ms(100);                                                   // 初始化失败 等待 100ms
  }
  
  if(1 != WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
  {
      while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
          "TCP",                                                              // 指定使用TCP方式通讯
          WIFI_SPI_TARGET_IP,                                                 // 指定远端的IP地址，填写上位机的IP地址
          WIFI_SPI_TARGET_PORT,                                               // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
          WIFI_SPI_LOCAL_PORT))                                               // 指定本机的端口号
      {
          // 如果一直建立失败 考虑一下是不是没有接硬件复位
          printf("\r\n Connect TCP Servers error, try again.");
          system_delay_ms(100);                                               // 建立连接失败 等待 100ms
      }
  }
    
  
  // 逐飞助手初始化 数据传输使用高速WIFI SPI
  seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
  
  #if(0 == INCLUDE_BOUNDARY_TYPE)
  // 发送总钻风图像信息(仅包含原始图像信息)
  seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, mt9v03x_image_pross3[0], MT9V03X_W, MT9V03X_H);
    
    
#elif(1 == INCLUDE_BOUNDARY_TYPE)
    // 发送总钻风图像信息(并且包含三条边界信息，边界信息只含有横轴坐标，纵轴坐标由图像高度得到，意味着每个边界在一行中只会有一个点)
    // 对边界数组写入数据
    for(i = 0; i < MT9V03X_H; i++)
    {
        x1_boundary[i] = 70 - (70 - 20) * i / MT9V03X_H;
        x2_boundary[i] = MT9V03X_W / 2;
        x3_boundary[i] = 118 + (168 - 118) * i / MT9V03X_H;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, mt9v03x_image_pross3[0], MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, MT9V03X_H, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);
    
    
#elif(2 == INCLUDE_BOUNDARY_TYPE)
    // 发送总钻风图像信息(并且包含三条边界信息，边界信息只含有纵轴坐标，横轴坐标由图像宽度得到，意味着每个边界在一列中只会有一个点)
    // 通常很少有这样的使用需求
    // 对边界数组写入数据
    for(i = 0; i < MT9V03X_W; i++)
    {
        y1_boundary[i] = i * MT9V03X_H / MT9V03X_W;
        y2_boundary[i] = MT9V03X_H / 2;
        y3_boundary[i] = (MT9V03X_W - i) * MT9V03X_H / MT9V03X_W;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(Y_BOUNDARY, MT9V03X_W, NULL, NULL ,NULL, y1_boundary, y2_boundary, y3_boundary);
    
    
#elif(3 == INCLUDE_BOUNDARY_TYPE)
    // 发送总钻风图像信息(并且包含三条边界信息，边界信息含有横纵轴坐标)
    // 这样的方式可以实现对于有回弯的边界显示
    j = 0;
    for(i = MT9V03X_H - 1; i >= MT9V03X_H / 2; i--)
    {
        // 直线部分
        xy_x1_boundary[j] = 34;
        xy_y1_boundary[j] = i;
        
        xy_x2_boundary[j] = 47;
        xy_y2_boundary[j] = i;
        
        xy_x3_boundary[j] = 60;
        xy_y3_boundary[j] = i;
        j++;
    }
    
    for(i = MT9V03X_H / 2 - 1; i >= 0; i--)
    {
        // 直线连接弯道部分
        xy_x1_boundary[j] = 34 + (MT9V03X_H / 2 - i) * (MT9V03X_W / 2 - 34) / (MT9V03X_H / 2);
        xy_y1_boundary[j] = i;
        
        xy_x2_boundary[j] = 47 + (MT9V03X_H / 2 - i) * (MT9V03X_W / 2 - 47) / (MT9V03X_H / 2);
        xy_y2_boundary[j] = 15 + i * 3 / 4;
        
        xy_x3_boundary[j] = 60 + (MT9V03X_H / 2 - i) * (MT9V03X_W / 2 - 60) / (MT9V03X_H / 2);
        xy_y3_boundary[j] = 30 + i / 2;
        j++;
    }
    
    for(i = 0; i < MT9V03X_H / 2; i++)
    {
        // 回弯部分
        xy_x1_boundary[j] = MT9V03X_W / 2 + i * (138 - MT9V03X_W / 2) / (MT9V03X_H / 2);
        xy_y1_boundary[j] = i;
        
        xy_x2_boundary[j] = MT9V03X_W / 2 + i * (133 - MT9V03X_W / 2) / (MT9V03X_H / 2);
        xy_y2_boundary[j] = 15 + i * 3 / 4;
        
        xy_x3_boundary[j] = MT9V03X_W / 2 + i * (128 - MT9V03X_W / 2) / (MT9V03X_H / 2);
        xy_y3_boundary[j] = 30 + i / 2;
        j++;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(XY_BOUNDARY, BOUNDARY_NUM, xy_x1_boundary, xy_x2_boundary, xy_x3_boundary, xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);
    
    
#elif(4 == INCLUDE_BOUNDARY_TYPE)
    // 发送总钻风图像信息(并且包含三条边界信息，边界信息只含有横轴坐标，纵轴坐标由图像高度得到，意味着每个边界在一行中只会有一个点)
    // 对边界数组写入数据
    for(i = 0; i < MT9V03X_H; i++)
    {
        x1_boundary[i] = 70 - (70 - 20) * i / MT9V03X_H;
        x2_boundary[i] = MT9V03X_W / 2;
        x3_boundary[i] = 118 + (168 - 118) * i / MT9V03X_H;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, NULL, MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, MT9V03X_H, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);
    
    
#endif
    
#endif
  wireless_sem = rt_sem_create("wireless_sem", 0, RT_IPC_FLAG_PRIO);
  wireless_tid = rt_thread_create("wireless_tid", wireless_entry, RT_NULL, 1024, 3, 30);
  //开启进程
  if(RT_NULL != wireless_tid)
  {
      rt_thread_startup(wireless_tid);
      wifi_ready=1;
  }
  else													// 线程创建失败
  {
      rt_kprintf("wireless_tid thread create ERROR.\n");
      return;
  }
}
