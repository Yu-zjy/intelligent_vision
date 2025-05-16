#include "zf_common_headfile.h"
#include "display.h"

//�ź�������
rt_sem_t display_sem;
uint8 maxLable = 0;

extern unsigned char csimenu_flag[4];//����ͷ��־λ,������ͨ��UI��
extern uint8 * ImageP;
extern uint8 NOpoint;
extern uint16 real_pointx,real_pointy;
extern uint8 imageProcess_flag;

void display_entry(void *parameter)
{
    while(1)
    {
      //�ź�����Imagez������ͷ�
      rt_sem_take(display_sem, RT_WAITING_FOREVER);
        //���Կ�����С��ʾ�������ͬ����ʾһЩ����
      
        if(csimenu_flag[0]==1) 
        {
          ips114_displayimage03x(ImageP, 240,135);//��ʾ�Ҷ�����ͷ ͼ��
        }
        else if(csimenu_flag[1]==1) 
        {
          ips114_displayimage03x(mt9v03x_image_pross3[0],240,135);//��ʾ�Ҷ�����ͷ ͼ��
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
//            findCentral(maxLable,mt9v03x_csi_image_pross2[0]);//������λ�洢����Ѱ��
//          }
          ips114_displayimage03x(mt9v03x_image_pross1[0], 240,135);//��ʾ���������ͷ ͼ��
          
//          if(NOpoint != 1)//����Ч����
//          {
            ipsDraw_bline(mt9v03x_image_pross2[0]);
//            NOpoint=1;
//          }
          
//          lcd_clear(BLACK);
//          lcd_showfloat(0,1,pic_div,3,2);
          ips114_show_uint(0,0,THRESHOLDforJunmpPoint,3);
          ips114_show_uint(0,1,real_pointx,3);
          ips114_show_uint(0,2,real_pointy,3);

            
          //����ͨ��  ����λ�洢��
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
    
    //��ʼ����Ļ
    ips114_init();     	//��ʼ��TFT��Ļ
    ips114_show_string(0,0,"SEEKFREE ips200_init");
    ips114_show_string(0,1,"Initializing...");
    //UI��ʼ��
    //menu_display();
    //�����ź�������ʾͼ���������� ������ͷCSI���ʱ�ͷ�
    display_sem = rt_sem_create("display_sem", 0, RT_IPC_FLAG_PRIO);
    //������ʾ�߳� ���ȼ�����Ϊ31
    tid = rt_thread_create("display", display_entry, RT_NULL, 1024, 3, 30);//���ȼ���ͣ���idle�߳� ���ǿ��Ա���һֱ����
    
    //������ʾ�߳�
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
    else													// �̴߳���ʧ��
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
5*7����  20cmΪ���  �� 25*35�������ڵ�֮����3�����ص�
100*140�����ص�ķ�Χ
  */
  uint8 debugStr[20];
  uint8 i;
  Coordinate * p=positionSet;
  ips114_full(RGB565_WHITE);
  //��ʾ����
  ips114_show_uint(100,0,num,3);
  //����
  ips114_draw_line(10,14,150,14,LINECOLOR);
  ips114_draw_line(10,114,150,114,LINECOLOR);
  ips114_draw_line(10,14,10,114,LINECOLOR);
  ips114_draw_line(150,14,150,114,LINECOLOR);
  //��Ŀ���ʹ��ڴ�ӡ
  uart_write_string(UART_4, "POSITION_GET over...\n");
  for(i=1;i<num+1;i++)
  {
    ips114_draw_point((uint16)(10+4 *(p+i)->X *100/20),xAxisymmetry(ips114_y_max/2,(uint16)(14+4 *(p+i)->Y *100/20)),RGB565_BLACK);
    wireless_uart_send_buff(debugStr, sprintf(debugStr, "num%d\rx=%fmy=%fm\n",i,(p+i)->X,(p+i)->Y ));
  }
}
