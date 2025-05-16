#include "zf_common_headfile.h"
#include "display.h"

//信号量创建
rt_sem_t display_sem;
uint8 maxLable = 0;

extern unsigned char csimenu_flag[4];//摄像头标志位,按键修通过UI改
extern uint8 * ImageP;
extern uint8 NOpoint;
extern uint16 real_pointx,real_pointy;
extern uint8 imageProcess_flag;

void display_entry(void *parameter)
{
    while(1)
    {
      //信号量在Imagez中完成释放
      rt_sem_take(display_sem, RT_WAITING_FOREVER);
        //可以考虑缩小显示区域便于同步显示一些参数
      
        if(csimenu_flag[0]==1) 
        {
          ips114_displayimage03x(ImageP, 240,135);//显示灰度摄像头 图像
        }
        else if(csimenu_flag[1]==1) 
        {
          ips114_displayimage03x(mt9v03x_image_pross3[0],240,135);//显示灰度摄像头 图像
          ips114_show_uint(0,0,THRESHOLD,3);
          ips_DrawCross(real_pointx, real_pointy); 
        }
        else if(csimenu_flag[2]==1) 
        {
          find_jump_point(ImageP,mt9v03x_image_pross1[0]);
          connect_domain2(mt9v03x_image_pross1[0],mt9v03x_image_pross2[0]);
          maxLable=0;
          if(
              (garageEnter.max1XRight - garageEnter.max1XLeft) > 30 && \
              (garageEnter.max1Ydown - garageEnter.max1Yup) > 40 && \
                ((garageEnter.max1Yupdiff > 45)||(garageEnter.max1Ydowndiff > 45))
            )
          {
            maxLable = garageEnter.max1Lable;
            real_pointx = (garageEnter.max1XRight/2 + garageEnter.max1XLeft/2);
            real_pointy = (garageEnter.max1Ydown/2 + garageEnter.max1Yup/2);
            ips114_show_uint(0,3,garageEnter.max1YupdiffX,3);
            ips114_show_uint(0,4,garageEnter.max1YdowndiffX,3);
          }
          else if(
              (garageEnter.max2XRight - garageEnter.max2XLeft) > 30 && \
              (garageEnter.max2Ydown - garageEnter.max2Yup) > 40 && \
              ((garageEnter.max2Yupdiff > 45)||(garageEnter.max2Ydowndiff > 45))
            )
          {
            maxLable = garageEnter.max2Lable;
            real_pointx = (garageEnter.max2XRight/2 + garageEnter.max2XLeft/2);
            real_pointy = (garageEnter.max2Ydown/2 + garageEnter.max2Yup/2);
          ips114_show_uint(0,3,garageEnter.max2YupdiffX,3);
          ips114_show_uint(0,4,garageEnter.max2YdowndiffX,3);
          }
          else if(
              (garageEnter.max3XRight - garageEnter.max3XLeft) > 30 && \
              (garageEnter.max3Ydown - garageEnter.max3Yup) > 40 && \
              ((garageEnter.max3Yupdiff > 45)||(garageEnter.max3Ydowndiff > 45))
            )
          {
            maxLable = garageEnter.max3Lable;
            real_pointx = (garageEnter.max3XRight/2 + garageEnter.max3XLeft/2);
            real_pointy = (garageEnter.max3Ydown/2 + garageEnter.max3Yup/2);
          ips114_show_uint(0,3,garageEnter.max3YupdiffX,3);
          ips114_show_uint(0,4,garageEnter.max3YdowndiffX,3);
          }
          
//          if(maxLable!=0)
//          {
//            findCentral(maxLable,mt9v03x_csi_image_pross2[0]);//不带复位存储区的寻找
//          }
          ips114_displayimage03x(mt9v03x_image_pross1[0], 240,135);//显示跳变点摄像头 图像
          
//          if(NOpoint != 1)//有有效亮点
//          {
            ipsDraw_bline(mt9v03x_image_pross2[0]);
//            NOpoint=1;
//          }
          
//          lcd_clear(BLACK);
//          lcd_showfloat(0,1,pic_div,3,2);
          ips114_show_uint(0,0,THRESHOLDforJunmpPoint,3);
          ips114_show_uint(0,1,real_pointx,3);
          ips114_show_uint(0,2,real_pointy,3);

            
          //画连通域  并复位存储区
          for(uint8 j=0;j<MT9V03X_H;j++)
          {
            for(uint8 i=0;i<MT9V03X_W;i++)
            {
              if(*(mt9v03x_image_pross2[0]+j*MT9V03X_W+i)== maxLable)
              {
//                lcd_drawpoint(STANDPOINTX(i),STANDPOINTY(j),BLUE);
              }
              *(mt9v03x_image_pross2[0]+j*MT9V03X_W+i)= 0;
            }
           }
        }    
    }
}






void display_init(void)
{
    rt_thread_t tid;
    
    //初始化屏幕
    ips114_init();     	//初始化TFT屏幕
    ips114_show_string(0,0,"SEEKFREE ips200_init");
    ips114_show_string(0,1,"Initializing...");
    //UI初始化
    //menu_display();
    //创建信号量，表示图像处理完成与否 在摄像头CSI完成时释放
    display_sem = rt_sem_create("display_sem", 0, RT_IPC_FLAG_PRIO);
    //创建显示线程 优先级设置为31
    tid = rt_thread_create("display", display_entry, RT_NULL, 1024, 3, 30);//优先级最低，和idle线程 但是可以保持一直运行
    
    //启动显示线程
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
    else													// 线程创建失败
    {
        rt_kprintf("display thread create ERROR.\n");
        return ;
    }
}

uint16 xAxisymmetry( uint16 ynum, uint16 yCoordinate)
{
  if(yCoordinate>ynum) return ynum-(yCoordinate-ynum);
  else if(yCoordinate<ynum) return ynum+(-yCoordinate+ynum);
  else if(yCoordinate==ynum) return yCoordinate;
  return 0;
}

void ipsDisplayCoordinateGet(uint8 num, Coordinate * positionSet)
{
  /*
5*7场地  20cm为间隔  即 25*35个点相邻点之间间隔3个像素点
100*140个像素点的范围
  */
  uint8 debugStr[20];
  uint8 i;
  Coordinate * p=positionSet;
  ips114_full(RGB565_WHITE);
  //显示点数
  ips114_show_uint(100,0,num,3);
  //画框
  ips114_draw_line(10,14,150,14,LINECOLOR);
  ips114_draw_line(10,114,150,114,LINECOLOR);
  ips114_draw_line(10,14,10,114,LINECOLOR);
  ips114_draw_line(150,14,150,114,LINECOLOR);
  //画目标点和串口打印
  uart_write_string(UART_4, "POSITION_GET over...\n");
  for(i=1;i<num+1;i++)
  {
    ips114_draw_point((uint16)(10+4 *(p+i)->X *100/20),xAxisymmetry(ips114_y_max/2,(uint16)(14+4 *(p+i)->Y *100/20)),RGB565_BLACK);
    wireless_uart_send_buff(debugStr, sprintf(debugStr, "num%d\rx=%fmy=%fm\n",i,(p+i)->X,(p+i)->Y ));
  }
}
