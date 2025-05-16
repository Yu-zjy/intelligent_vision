#include "image.h"


/*
*图像处理相关
*注意摄像头是188*120 LCD是160*128 显示在屏幕上的图像进行了缩放
*一张图像大小22KB左右
*/
int16 debug_stardline=start_line;
uint8 THRESHOLD = 120; 
uint8 THRESHOLD_PIC = 110;
findline_TypeDef Findline = {0};
extern uint8 find_pic;//1左边，2右边
uint8 curve_pic=0;//U弯图片
uint8 road_type;
uint8 curve_type=0;
uint8 cross_task=0;//1十字后左转，2十字后右转
uint8 roundabout_type=0;
uint8 startline_stop_flag = 0;//起跑斑马线检测标志位，0起跑，1检测，2停止
float roundstart_angle;
//uint8 debug_num;
  uint8 xerr;
float varian=0;
//处理后的图像存储区
AT_DTCM_SECTION_ALIGN(uint8 mt9v03x_image_pross_store1[MT9V03X_H][MT9V03X_W], 64);
AT_DTCM_SECTION_ALIGN(uint8 mt9v03x_image_pross_store2[MT9V03X_H][MT9V03X_W],64);
AT_DTCM_SECTION_ALIGN(uint8 mt9v03x_image_pross_store3[MT9V03X_H][MT9V03X_W],64);   //简单二值化后图像
AT_DTCM_SECTION_ALIGN(uint8 mt9v03x_image_pross_store4[MT9V03X_H][MT9V03X_W],64);   //原版连通域处理后的图像
AT_DTCM_SECTION_ALIGN(uint8 mt9v03x_image_storePoll[MT9V03X_H][MT9V03X_W], 64);

//用户访问图像数据直接访问这个指针变量就可以
//访问方式非常简单，可以直接使用下标的方式访问
//例如访问第10行 50列的点，mt9v03x_csi_image[10][50]就可以了
uint8 (*mt9v03x_image_pross1)[MT9V03X_W] =      mt9v03x_image_pross_store1;
uint8 (*mt9v03x_image_pross2)[MT9V03X_W] =      mt9v03x_image_pross_store2;
uint8 (*mt9v03x_image_pross3)[MT9V03X_W] =      mt9v03x_image_pross_store3;
uint8 (*mt9v03x_image_pross4)[MT9V03X_W] =      mt9v03x_image_pross_store4;
uint8 (*mt9v03x_image_store)[MT9V03X_W] =       mt9v03x_image_storePoll;

uint8 angle_correct=0;
uint8 angle_correct_finish=0;
uint8 imageProcess_flag=0;
//uint8 imageProcessFinish_flag=0;
//uint8 imageProcess_found=0;
//uint8 task_finish_flag=0;
//
//int16 width[120]={0,46,46,47,48,48,49,50,50,51,//0-9
//    52,52,54,54,54,56,56,57,58,58,59,60,61,61,62,63,63,64,65,65,//10-29
//    66,67,67,68,69,70,70,71,72,72,73,74,74,74,76,76,77,78,78,79,//30-49
//    80,81,81,81,83,83,83,85,85,86,86,87,88,88,90,90,91,92,92,93,//50-69
//    93,94,95,95,97,97,97,99,99,100,101,101,102,103,103,104,104,105,106,106,//70-89
//    108,108,108,110,110,110,111,112,113,113,114,111,113,113,114,115,115,116,117,117,//90-109
//    118,119,119,120,121,122,153,0,0,0//110-119
//};//赛道宽

int16 width[120]={  
    0, 54, 55, 57, 58, 60, 60, 62, 64, 65,   
    66, 68, 69, 71, 72, 73, 75, 77, 78, 79,   
    81, 82, 84, 86, 86, 88, 90, 91, 92, 94,   
    95, 97, 98, 99, 100, 102, 104, 105, 106, 108,   
    109, 111, 113, 113,115, 117, 118, 120, 122, 122, 124,   
    125, 127, 128, 129, 131, 131, 131, 131, 131, 131,   
    131, 131, 131, 131, 131, 131, 131, 131, 131, 131,   
    131, 131, 131, 131, 131, 131, 131, 131, 131, 131,  
    131, 131, 131, 131, 131, 131, 131, 131, 131, 131,  
    131, 131, 131, 131, 131, 131, 131, 131, 131 
};  
uint8 otsuThreshold(uint8 *image);

void err_calculate(void);
void findline(void);
void err_calculate2(void);
void start_clearjump(void);
void left_width_findline(void);
void right_width_findline(void);
void midle_findline(void);
void crossing_connect(void);
void EmergencyStop(void);
void road_check(void);
void road_check2(void);
uint8 lround_identify1(void);
uint8 rround_identify1(void);
uint8 lround_identify2(void);
uint8 rround_identify2(void);
uint8 round_identify3(void);
void lround_check(void);
void rround_check(void);
void crossing_check(void);
void jump_time(void);//斑马线检测，跳变点数量

void Find_pic(uint8 *p_pross);
//void Pic_correct(uint8 *p_pross);
//void find_pic_none(uint8 *p_pross);
//void pic_locate(uint8 *p_pross);
//void pic_locate2(uint8 *p_pross);
//void pic_locate_angle(uint8 *p_pross);

void quickSort(uint8 data[], int low, int high);
void FilterIndependentNoise(uint8 *pic,uint8 *processed_pic);
void binaryzationOperate_simple(uint8 *p,uint8 *p_pross,uint8 *p_pic);
void display_line(uint8 *p_pross);

uint8 measure_track_width(int16 *target_width);
int16 calibrated_width[120] = {0};
//信号量创建
rt_sem_t image_sem;
extern rt_sem_t wireless_sem;
extern CARSTATUS_enum stateTop;
//unsigned char csimenu_flag[4];//摄像头标志位,按键修通过UI改
uint8 * ImageP=NULL;

extern float angleSet;
//extern float SetX,SetY; 
extern uint8 stopFlag;
extern uint8 wifi_ready;
extern float before_locpicx,before_locpicy;

void image_entry(void *parameter)
{
//  uint16 pic_midpoint_x,pic_midpoint_y;
  while(1)
  {
    //获取摄像头图像的信息量
    rt_sem_take(image_sem, RT_WAITING_FOREVER);
    
    if(mt9v03x_finish_flag)
    {
      mt9v03x_finish_flag=0;
      binaryzationOperate_simple((uint8 *)mt9v03x_image,mt9v03x_image_pross3[0],mt9v03x_image_pross2[0]);
      //measure_track_width(calibrated_width);
            //ips200_displayimage03x((const uint8 *)mt9v03x_image, 240, 300);
//            ips200_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, 240, 180, 0);
            
      switch(imageProcess_flag)
      {
      case 1://正常巡线
        jump_time();
        findline();
//       int16 i,j;
//      for(i=119;i>107;i--)
        
      
        if(roundabout_type==0&&startline_stop_flag<2)
        {
          road_check2();
        }
//        else stopFlag=1;
        if(stateTop==FINDLINE&&find_pic==0&&road_type<2&&startline_stop_flag<2) Find_pic(mt9v03x_image_pross3[0]);
        EmergencyStop();
        if(Findline.loseflag==1)
        {
          road_type = LOSE;
        }
        else
        {
          if(startline_stop_flag>1) 
          {
            road_type=BLOCK;
            start_clearjump();
            Findline.sight=cur_sight;
//            if(Findline.rightline[straight_sight]-Findline.leftline[straight_sight]>width[straight_sight]-10) Findline.sight=straight_sight;
//            else if(Findline.rightline[cur_sight]-Findline.leftline[cur_sight]>width[cur_sight]-10) Findline.sight=cur_sight;
//            else Findline.sight=40;
          }
          else if(road_type==STRAIGHT) 
          {
            Findline.sight = straight_sight;
          }
          else if(find_pic&&roundabout_type==0) Findline.sight=105;
          else
          {
            if(road_type==ROUNDABOUTL)
            {
              Findline.sight = cur_sight;
              lround_check();
              if(roundabout_type==10||roundabout_type<5) right_width_findline();
              else left_width_findline();
              if(roundabout_type>5&&roundabout_type<10) Findline.sight=105;
            }
            else if(road_type==ROUNDABOUTR)
            {
              Findline.sight = cur_sight;
              rround_check();
              if(roundabout_type==10||roundabout_type<5) left_width_findline();
              else right_width_findline();
              if(roundabout_type>5&&roundabout_type<10) Findline.sight=105;
            }
            else if(road_type==CROSSING)
            {
              crossing_check();
              crossing_connect();
              if(roundabout_type==6) Findline.sight=105;
            }
            else 
            {
              Findline.sight=100;
            }
          }
//        display_line(mt9v03x_image_pross3[0]);
        err_calculate(); 
      }
      break;
    default:
      break;
      }
//      if(wifi_ready) rt_sem_release(wireless_sem);
//      int16 i,j;
//      for(i=119;i>107;i--)
//        for(j=0;j<159;j++)
//        {
//          mt9v03x_image_pross3[i][j]=0;
//        }
    }
  }
}

void image_init(void)
{
    rt_thread_t tid;    
    
    THRESHOLD=180;
    image_sem = rt_sem_create("image_sem", 0, RT_IPC_FLAG_PRIO);
       
    //创建显示线程 优先级设置为5
    tid = rt_thread_create("image", image_entry, RT_NULL, 4096, 6, 20);//优先级很高，保证图像被处理
    Findline.sight = straight_sight;
    
    //启动显示线程
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
    else													// 线程创建失败
    {
        rt_kprintf("image thread create ERROR.\n");
        return ;
    }
        
}

#define GrayScale   256
//二值化 调用大津法
//根据强光照下的高亮度目标的改良版
uint8 otsuThreshold(uint8 *image) // 注意计算阈值的一定要是原图像
{
  uint16 width = MT9V03X_H;
  uint16 height = MT9V03X_W;
  int pixelCount[GrayScale];               // 每个灰度像素计数数组
  float pixelPro[GrayScale];               // 每个灰度像素概率数组
  int i, j, pixelSum = width * height / 4; // 总像素点数除4是因为要减少计算量
  uint8 threshold = 0;                     // 初始化阈值
  uint8 *data = image;                     // 指向图像数据的指针
  for (i = 0; i < GrayScale; i++)          // 统计数组初始化
  {
    pixelCount[i] = 0;
    pixelPro[i] = 0;
  }

  uint32 gray_sum = 0; // 灰度等级累积
  // 统计灰度级中每个像素在整幅图像中的个数
  for (i = 0; i < height; i += 2)
  {
    for (j = 0; j < width; j += 2)
    {
      pixelCount[(int)data[i * width + j]]++; // 将当前的点的像素值作为计数数组的下标，以统计每个灰度等级的像素点数
      gray_sum += (int)data[i * width + j];   // 灰度值总和
    }
  }
  // 计算每个像素值的点在整幅图像中的比例
  for (i = 0; i < GrayScale; i++)
  {
    pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }
  // 遍历灰度级[gray_start,gray_stop ] start/stop是根据实际情况人工确定的遍历起点/重点，以减小计算量
  uint8 gray_start = 0, gray_stop = 255;
  float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
  w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
  for (j = gray_start; j < gray_stop; j++)
  {
    w0 += pixelPro[j];        // 背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
    u0tmp += j * pixelPro[j]; // 背景部分 每个灰度值的点的比例 *灰度值

    w1 = 1 - w0;
    u1tmp = gray_sum / pixelSum - u0tmp;

    u0 = u0tmp / w0;   // 背景平均灰度
    u1 = u1tmp / w1;   // 前景平均灰度
    u = u0tmp + u1tmp; // 全局平均灰度
    deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
    if (deltaTmp > deltaMax)
    {
      deltaMax = deltaTmp;
      threshold = j;
    }
    if (deltaTmp < deltaMax)
    {
      break;
    }
  }
  return threshold;
}

/**  
 * @brief 测量静止状态下直道的宽度数组  
 * @param target_width  输出参数，用于存储测量后的宽度数组（长度需≥120）  
 * @return uint8       结果状态：0-未完成，1-完成  
 */  
uint8 measure_track_width(int16 *target_width) {  
    // 检查参数合法性  
    if (target_width == NULL) {  
        return 0; // 参数错误，返回0  
    }  

    // 清零目标数组  
    memset(target_width, 0, sizeof(int16) * 120);  

    // 进行宽度测量  
    for (int i = 0; i < 120; i++) {  
        // 检查当前行是否有效（左右边界均被检测到）  
        if (Findline.leftlineflag[i] && Findline.rightlineflag[i]) {  
            target_width[i] = (Findline.rightline[i] - Findline.leftline[i]); // 计算宽度  
        } else {  
            target_width[i] = 0; // 如果无效则设为0  
        }  
    }  

    return 1; // 测量完成  
}  

/*******************支持函数，不向外提供接口*************************/
/*--------------------巡线---------------------*/
void err_calculate(void)
{
  int16 i;
  if((find_pic&&roundabout_type==0)||(roundabout_type>5&&roundabout_type<10&&cross_task==0)
     ||road_type==CURVE||(roundabout_type==6&&cross_task)) Findline.angle_err=(float)(Findline.midline[Findline.sight-30]);

  //  else if(roundabout_type==6)
//  {
//    if(road_type==ROUNDABOUTL) Findline.angle_err=(float)(Findline.midline[Findline.sight]-10);
//    else Findline.angle_err=(float)(Findline.midline[Findline.sight]+10);
//  }
  else Findline.angle_err=(float)(Findline.midline[Findline.sight]);
  xerr = Findline.angle_err- midle_lineloc;
  Findline.angle_err-= midle_lineloc;
  Findline.angle_err=Findline.angle_err/30.0f;
  if(Findline.angle_err<0)
  {
    Findline.angle_err=-atan(-Findline.angle_err)*180.0/PI;
  }
  else
  {
    Findline.angle_err=atan(Findline.angle_err)*180.0/PI;
  }
  if(fabs(Findline.angle_err)<=5)
  {
    Findline.angle_err=0;
    if(angle_correct) angle_correct_finish=1;
  }
  for(i=4;i>=1;i--)//软件更新
  {
    Findline.err[i] = Findline.err[i - 1];
  }
  Findline.err[0]=0;

  for(i =Findline.sight ; i > Findline.sight-5; i--)// 弯道；摄像头前瞻修改位置， 通过修改i的范围来控制前瞻
  { 
    Findline.err[0]+= (float)(Findline.midline[i] - midle_lineloc);
  }
  Findline.err[0] = Findline.err[0]/(5*CAM_REA);

  if(Findline.err[0] * Findline.err[1] < 0 && fabs(Findline.err[0] - Findline.err[1])> midle_lineloc)
  {
    Findline.err[0] = Findline.err[1];
  }
  
  angleSet =gyro.TurnAngle_Integral + Findline.angle_err;
  Angle_adapt();
}

void findline(void)//一般巡线
{
  int16 i = 0, j = 0;
  //参数初始化
  Findline.midline[start_line+1] = midle_lineloc;
  Findline.leftline[start_line+1] = 3;
  Findline.rightline[start_line+1] = imgw;
  Findline.leftstartpoint = 0;
  Findline.rightstartpoint = 0;
  Findline.straightend=0;
  Findline.endline = 0;
  for(i = start_line; i > 0; i--)
  {
    Findline.midline[i] = midle_lineloc;
    Findline.leftline[i] = 3;
    Findline.rightline[i] = imgw;
    Findline.leftlineflag[i] = 0;
    Findline.rightlineflag[i] = 0;

    if(Findline.midline[i + 1] > midle_lineloc)j = Findline.midline[i + 1] > imgw ? imgw : Findline.midline[i + 1];
    else j = Findline.midline[i + 1] < 4 ? 4 : Findline.midline[i + 1];  //先找到中间位置，从中间向左边开始找
    for(; j > 3; j--)
    {
      if(mt9v03x_image_pross3[i][j] == 255 && mt9v03x_image_pross3[i][j - 1] == 255 && mt9v03x_image_pross3[i][j - 2] == 0 &&mt9v03x_image_pross3[i][j - 3] == 0)
      {
        if(Findline.leftstartpoint == 0) Findline.leftstartpoint = i;//最下方开始有边界的行数
        Findline.leftline[i] = j - 1;
        Findline.leftlineflag[i] = 1;
        break;
      }
    }

    if(Findline.midline[i + 1] > midle_lineloc)j = Findline.midline[i + 1] > imgw ? imgw : Findline.midline[i + 1];
    else j = Findline.midline[i + 1] < 4 ? 4 : Findline.midline[i + 1];
    for(; j < imgw; j++)//从中间向右边开始找
    {
      if(mt9v03x_image_pross3[i][j] == 255 && mt9v03x_image_pross3[i][j + 1] == 255 && mt9v03x_image_pross3[i][j + 2] == 0 && mt9v03x_image_pross3[i][j + 3] == 0)
      {
        if(Findline.rightstartpoint == 0) Findline.rightstartpoint = i;
        Findline.rightline[i] = j + 1;
        Findline.rightlineflag[i] = 1;
        break;
      }
    }
    Findline.midline[i] = (Findline.leftline[i] + Findline.rightline[i]) / 2;
//    Findline.line_with_test[i]=Findline.rightline[i]-Findline.leftline[i];
    //边线都没找到
    if((Findline.leftlineflag[i] == 0) && (Findline.rightlineflag[i] == 0))
    {
      Findline.midline[i] = Findline.midline[i + 1];
    }

    //找顶点
    if(Findline.midline[i + 1] > midle_lineloc)j = Findline.midline[i + 1] > imgw ? imgw : Findline.midline[i + 1];
    else j = Findline.midline[i + 1] < 4 ? 4 : Findline.midline[i + 1];
    if( mt9v03x_image_pross3[i - 1][j] == 0 && (mt9v03x_image_pross3[i - 1][j - 2] == 0 || mt9v03x_image_pross3[i - 1][j + 2] == 0) )
    {
      Findline.endline = i;
      break;
    }
    if(Findline.straightend==0&&mt9v03x_image_pross3[i - 1][midle_lineloc] == 0 && (mt9v03x_image_pross3[i - 1][midle_lineloc - 2] == 0 || mt9v03x_image_pross3[i - 1][midle_lineloc + 2] == 0))
    {
      Findline.straightend=i;
    }
  }
}

void start_clearjump(void)
{
  int16 i;
  for(i=start_line-1;i>straight_sight;i--)
  {
    if(Findline.leftlineflag[i]&&Findline.rightlineflag[i]&&Findline.rightline[i]-Findline.leftline[i]<width[i]-5)
    {
      Findline.leftlineflag[i]=0;
      Findline.rightlineflag[i]=0;
      Findline.midline[i]=Findline.midline[i+1];
    }
  }
}

void left_width_findline(void)//半宽巡线
{
  int16 i,j;
  for(i=start_line;i>Findline.endline;i--)
  {
    Findline.midline[i]=Findline.leftline[i]+width[i]/2;
    if(roundabout_type>6&&roundabout_type<10) Findline.midline[i]+=15;
    j=Findline.midline[i];
    if( mt9v03x_image_pross3[i - 1][j] == 0 && (mt9v03x_image_pross3[i - 1][j - 2] == 0 || mt9v03x_image_pross3[i - 1][j + 2] == 0) )
    {
      Findline.endline = i;
      break;
    }
  }
}

void right_width_findline(void)
{
  int16 i,j;
  for(i=start_line;i>Findline.endline;i--)
  {
    Findline.midline[i]=Findline.rightline[i]-width[i]/2;
    if(roundabout_type>6&&roundabout_type<10) Findline.midline[i]-=15;
    j=Findline.midline[i];
    if( mt9v03x_image_pross3[i-1][j] == 0 && (mt9v03x_image_pross3[i-1][j-2] == 0 || mt9v03x_image_pross3[i-1][j+2] == 0) )
    {
      Findline.endline = i;
      break;
    }
  }
}

void midle_findline(void)
{
  int16 i,j;
  int16 mid_min=0;
  
  
  i=Findline.sight+1;
  for(j=3;j<imgw;j++)//如果入十字偏了，则寻找较小偏角的直道
  {
    if(mt9v03x_image_pross3[i][j] == 255&&mt9v03x_image_pross3[i][j-1] == 255&&mt9v03x_image_pross3[i][j+1] == 255)
    {
      if(abs(j-midle_lineloc)<abs(mid_min-midle_lineloc))
      {
        mid_min=j;
        if(mid_min>=midle_lineloc)
        {
          break;
        }
      }
    }
  }
  Findline.rightlineflag[i] = 0;
  Findline.leftlineflag[i] = 0;
  for(j=mid_min;j<imgw;j++)
  {    
    if(mt9v03x_image_pross3[i][j] == 255 && mt9v03x_image_pross3[i][j+1] == 255 && mt9v03x_image_pross3[i][j+2] == 0 && mt9v03x_image_pross3[i][j+3] == 0)
    {
      Findline.rightline[i] = j + 1;
      Findline.rightlineflag[i] = 1;
      break;
    }
  }
  for(j=mid_min; j>3; j--)
  {
    if(mt9v03x_image_pross3[i][j] == 255 && mt9v03x_image_pross3[i][j-1] == 255 && mt9v03x_image_pross3[i][j-2] == 0 &&mt9v03x_image_pross3[i][j-3] == 0)
    {
      Findline.leftline[i] = j - 1;
      Findline.leftlineflag[i] = 1;
      break;
    }
  }
  Findline.midline[i] = (Findline.leftline[i] + Findline.rightline[i]) / 2;
    //边线都没找到
  if((Findline.leftlineflag[i] == 0) && (Findline.rightlineflag[i] == 0))
  {
    Findline.midline[i] = midle_lineloc;
  }
  
  
  for(i = Findline.sight; i > Findline.endline; i--)//找十字后的直道
  {
    Findline.midline[i] = midle_lineloc;
    Findline.leftline[i] = 3;
    Findline.rightline[i] = imgw;
    Findline.leftlineflag[i] = 0;
    Findline.rightlineflag[i] = 0;

    if(Findline.midline[i + 1] > midle_lineloc)j = Findline.midline[i + 1] > imgw ? imgw : Findline.midline[i + 1];
    else j = Findline.midline[i + 1] < 4 ? 4 : Findline.midline[i + 1];  //先找到中间位置，从中间向左边开始找
    for(; j > 3; j--)
    {
      if(mt9v03x_image_pross3[i][j] == 255 && mt9v03x_image_pross3[i][j-1] == 255 && mt9v03x_image_pross3[i][j-2] == 0 &&mt9v03x_image_pross3[i][j-3] == 0)
      {
        if(Findline.leftstartpoint == 0) Findline.leftstartpoint = i;//最下方开始有边界的行数
        Findline.leftline[i] = j - 1;
        Findline.leftlineflag[i] = 1;
        break;
      }
    }

    if(Findline.midline[i + 1] > midle_lineloc)j = Findline.midline[i + 1] > imgw ? imgw : Findline.midline[i + 1];
    else j = Findline.midline[i + 1] < 4 ? 4 : Findline.midline[i + 1];
    for(; j < imgw; j++)//从中间向右边开始找
    {
      if(mt9v03x_image_pross3[i][j] == 255 && mt9v03x_image_pross3[i][j+1] == 255 && mt9v03x_image_pross3[i][j+2] == 0 && mt9v03x_image_pross3[i][j+3] == 0)
      {
        if(Findline.rightstartpoint == 0) Findline.rightstartpoint = i;
        Findline.rightline[i] = j + 1;
        Findline.rightlineflag[i] = 1;
        break;
      }
    }
    Findline.midline[i] = (Findline.leftline[i] + Findline.rightline[i]) / 2;
    //边线都没找到
    if((Findline.leftlineflag[i] == 0) || (Findline.rightlineflag[i] == 0))
    {
      Findline.midline[i] = Findline.midline[i + 1];
    }

    //找顶点
    if(Findline.midline[i + 1] > midle_lineloc)j = Findline.midline[i + 1] > imgw ? imgw : Findline.midline[i + 1];
    else j = Findline.midline[i + 1] < 4 ? 4 : Findline.midline[i + 1];
    if( mt9v03x_image_pross3[i-1][j] == 0 && (mt9v03x_image_pross3[i - 1][j-2] == 0 || mt9v03x_image_pross3[i-1][j+2] == 0) )
    {
      Findline.endline = i;
      break;
    }
    if(Findline.straightend==0&&mt9v03x_image_pross3[i-1][midle_lineloc] == 0)
    {
      if((mt9v03x_image_pross3[i-1][midle_lineloc-2]==0&&mt9v03x_image_pross3[i-1][midle_lineloc-4]==0)||(mt9v03x_image_pross3[i-1][midle_lineloc+2]==0&&mt9v03x_image_pross3[i-1][midle_lineloc+4]==0)) Findline.straightend=i;
    }
  }
}

void crossing_connect(void)
{
  uint8 lfound_jump,rfound_jump,lost_num,find_num;
  int16 i;
  Findline.ljumppoint=0;
  Findline.rjumppoint=0;
  lfound_jump=0;
  rfound_jump=0;
  lost_num=0;
  find_num=0;
  for(i=start_line;i>Findline.endline;i--)//寻找左上拐点
  {
    if(Findline.leftlineflag[i]==0||abs(Findline.leftline[i+1]-Findline.leftline[i])>5)
    {
      lost_num++;
      find_num=0;
    }
    else if(Findline.leftline[i]>10&&lost_num>10)
    {
      find_num++;
    }
    else 
    {
      lost_num=0;
      find_num=0;
    }
    if(lost_num>10&&find_num>5)
    {
      Findline.ljumppoint=i;
      if(i>15&&i<100) lfound_jump=1;
      break;  
    }
  }
  lost_num=0;
  find_num=0;
  for(i=start_line;i>Findline.endline;i--)//寻找右上拐点
  {
    if(Findline.rightlineflag[i]==0||abs(Findline.rightline[i]-Findline.rightline[i+1])>5)
    {
      lost_num++;
      find_num=0;
    }
    else if(Findline.rightline[i]<150&&lost_num>10)
    {
      find_num++;
    }
    else 
    {
      lost_num=0;
      find_num=0;
    }
    if(lost_num>10&&find_num>5)
    {
      Findline.rjumppoint=i;
      if(i>15&&i<100) rfound_jump=1;
      break;
    }
  }
//  if(lfound_jump==1&&rfound_jump==0)
//  { 
//    Findline.sight=Findline.ljumppoint-5;
//    left_width_findline();
//  }
//  else if(lfound_jump==0&&lfound_jump==1)
//  {
//    Findline.sight=Findline.rjumppoint-5;
//    right_width_findline();
//  }
  if(lfound_jump==1&&rfound_jump==1)
  {
    Findline.sight= Findline.rjumppoint<Findline.ljumppoint?Findline.rjumppoint:Findline.ljumppoint;
    midle_findline();
  }
  else 
  {
    Findline.sight=85;
  }
}

// 丢线停车
void EmergencyStop(void)
{
    //思路：丢线时近处的几行大部分都是黑色的，只要这部分的黑色像素累计大于一个阈值就认为丢线
    int16 i = 0, j = 0, count_black = 0;
    for(i = start_line; i > start_line-3; i--){
        for(j = 130; j >= 30; j--){
            if(mt9v03x_image_pross3[i][j] == 0){
                count_black = count_black + 1;
            }
        }
    }
    if(count_black > 200 )
    {
        Findline.loseflag = 1;
    }
    else{
        Findline.loseflag = 0;
    }
}

void road_check(void)
{
    int16 i;
    uint8 lbreak_num=0,rbreak_num=0;
    uint8 left_all_w,right_all_w;
    left_all_w = 1;
    right_all_w = 1;
    for(i=cur_sight;i>Findline.straightend;i--)
    {
      if(Findline.rightlineflag[i]==0||(Findline.rightline[i]-Findline.rightline[i+1]>5)) lbreak_num++;
      else lbreak_num=0;
      if(lbreak_num>5) right_all_w=0;
      if(Findline.leftlineflag[i]==0||(Findline.leftline[i+1]-Findline.leftline[i]>5)) rbreak_num++;
      else rbreak_num=0;
      if(rbreak_num>5) left_all_w=0;
    }
    if(left_all_w == 0&&right_all_w==0)//两边均异常，为十字
    {
      road_type = CROSSING;
      return;
    }
    else
    {
      if(Findline.straightend>25&&Findline.endline>5) 
      {
        road_type = CURVE;
        if(Findline.midline[Findline.endline]<midle_lineloc) curve_type=1;
        else curve_type=2;
        return;
      }
      uint8 n,m;
      n=0;
      m=0;
      for(i=start_line-1;i>=round_sight;i--)//先正常后变宽可能为环岛
      {
        if(abs(Findline.rightline[i]-Findline.leftline[i])<(width[i]+2*WIDTH_ERR)) m++;
        if(abs(Findline.rightline[i]-Findline.leftline[i])>(width[i]+5*WIDTH_ERR)&&m>5) 
        {
          n++;
        }
        if(n>10) break;
      }
      if(n>10)
      {
        if(left_all_w == 0&&right_all_w==1) 
        {
          if(lround_identify2()) road_type = ROUNDABOUTL;
          else road_type = STRAIGHT;
          return;
        }
        else if(left_all_w == 1&&right_all_w==0) 
        {
          if(rround_identify2()) road_type = ROUNDABOUTR;
          else road_type = STRAIGHT;
          return;
        }
        else
        {
          road_type=STRAIGHT;
          curve_type=0;
          return;
        }
      }
      else
      {
        road_type = STRAIGHT;
        curve_type=0;
        return;
      }
    }
}

void road_check2(void)
{
    int16 i,j;
    uint8 lbreak_point=0,rbreak_point=0;
    uint8 lbreak_num=0,rbreak_num=0;
    uint8 left_all_w,right_all_w;
    left_all_w = 1;
    right_all_w = 1;
    for(i=start_line-10;i>20;i--)
    {
      if(Findline.leftline[i]>Findline.leftline[i+7]&&Findline.leftline[i]>Findline.leftline[i+9]
         &&Findline.leftline[i]>Findline.leftline[i-2]&&Findline.leftline[i]>Findline.leftline[i-4])
      {
        if(2*Findline.leftline[i]-Findline.leftline[i-6]-Findline.leftline[i+6]>6&&i>Findline.endline+5)//拐点要在顶点之前
        {//寻找左拐点
          lbreak_point=i;
          break;
        }
      }
    }
    for(i=start_line-10;i>20;i--)
    {
      if(Findline.rightline[i]<Findline.rightline[i+7]&&Findline.rightline[i]<Findline.rightline[i+9]
         &&Findline.rightline[i]<Findline.rightline[i-2]&&Findline.rightline[i]<Findline.rightline[i-4])
      {
        if(Findline.rightline[i+6]+Findline.rightline[i-6]-2*Findline.rightline[i]>6&&i>Findline.endline+5)//拐点要在顶点之前
        {//寻找右拐点
          rbreak_point=i;
          break;
        }
      }
    }
    //找到一个拐点且未找到另一个拐点后找上拐点，斜入十字需要
    if(lbreak_point&&!rbreak_point)//找到左下未找到右下，找右上
    {
      int16 scan_end=Findline.leftline[lbreak_point];
      uint8 al_find=0;
      for(i=10;i<lbreak_point;i++)
      {
        al_find=0;
        for(j=156;j>scan_end-1;j--)//修正边线
        {
          if(mt9v03x_image_pross3[i][j+2]==0&&mt9v03x_image_pross3[i][j+1]==0&&mt9v03x_image_pross3[i][j]==255&&mt9v03x_image_pross3[i][j-1]==255)
          {
            scan_end=j;
            Findline.rightline[i]=j;
            Findline.rightlineflag[i]=1;
            al_find=1;
            break;
          }
        }
        if(!al_find)//检测是否为跳变点
        {
          if(Findline.rightline[i-2]<Findline.rightline[i]&&Findline.rightline[i-2]<Findline.rightline[i-8]) 
          {
            road_type = CROSSING;
            return;
          }
          break;
        }
      }
    }
    else if(rbreak_point&&!lbreak_point)//找到右下未找到左下，找左上
    {
      int16 scan_end=Findline.rightline[rbreak_point];
      uint8 al_find=0;
      for(i=10;i<rbreak_point;i++)
      {
        al_find=0;
        for(j=3;j<scan_end+1;j++)//修正边线
        {
          if(mt9v03x_image_pross3[i][j-2]==0&&mt9v03x_image_pross3[i][j-1]==0&&mt9v03x_image_pross3[i][j]==255&&mt9v03x_image_pross3[i][j+1]==255)
          {
            scan_end=j;
            Findline.leftline[i]=j;
            Findline.leftlineflag[i]=1;
            al_find=1;
            break;
          }
        }
        if(!al_find)//检测是否为跳变点
        {
          if(Findline.leftline[i-2]>Findline.leftline[i-8]&&Findline.leftline[i-2]>Findline.leftline[i]) 
          {
            road_type = CROSSING;
            return;
          }
          break;
        }
      }
    }  
    if(lbreak_point>35||rbreak_point>35)
    {
      if(lbreak_point)
      {
        for(i=lbreak_point;i>lbreak_point-15;i--)
        {
          if(abs(Findline.rightline[i]-Findline.leftline[i])>(width[i]+5*WIDTH_ERR)) lbreak_num++;
          else lbreak_num=0;
          if(lbreak_num>10) 
          {
            left_all_w=0;
            break;
          }
        }
      }
      if(rbreak_point)
      {
        for(i=rbreak_point;i>rbreak_point-15;i--)
        {
          if(abs(Findline.rightline[i]-Findline.leftline[i])>(width[i]+5*WIDTH_ERR)) rbreak_num++;
          else rbreak_num=0;
          if(rbreak_num>10) 
          {
            right_all_w=0;
            break;
          }
        }
      }
    }
    if(left_all_w == 0&&right_all_w==1&&lround_identify2()) 
    {
      road_type = ROUNDABOUTL;
      return;
    }
    else if(left_all_w == 1&&right_all_w==0&&rround_identify2()) 
    {
      road_type = ROUNDABOUTR;
      return;
    }
    else if(left_all_w == 0&&right_all_w==0)//两边均异常，为十字
    {
      road_type = CROSSING;
      return;
    }
//    else if(round_identify3())
//    {
//      roundabout_type=2;
//      return;
//    }
    else if(Findline.midline[Findline.endline+1]>60&&Findline.midline[Findline.endline+1]<100)
    {
      road_type = STRAIGHT;
      return;
    }
    else road_type=CURVE;
}

//通过环岛另一边线正常状态延续很远判断
uint8 lround_identify1(void)
{
  int16 rightlastpoint=0,i;
  for(i=start_line-1;i>round_sight;i--)
  {
    if(Findline.rightline[i]<midle_lineloc+width[i]/2-5)
    {
      rightlastpoint=i;
      break;
    }
  }
  if(rightlastpoint) return 0;
  else return 1;
}

uint8 rround_identify1(void)
{
  int16 leftlastpoint=0,i;
  for(i=start_line-1;i>round_sight;i--)
  {
    if(Findline.leftline[i]>midle_lineloc-width[i]/2+5)
    {
      leftlastpoint=i;
      break;
    }
  }
  if(leftlastpoint) return 0;
  else return 1;
}

//环岛另一边线找到线后不会丟线
uint8 lround_identify2(void)
{
  int16 i;
  uint8 r_find=0,r_lose=0,r_findbefore=0;
  for(i=start_line-5;i>start_line-15;i--)
  {
    if(Findline.leftlineflag[i]) 
    {
      r_findbefore=1;
      break;
    }
  }
  for(i=start_line-10;i>round_sight;i--)
  {
    if(Findline.rightlineflag[i])
    {
      if((!r_findbefore&&Findline.rightline[i]>153)||r_findbefore) r_find=i;//开始丟线则找到线应从边缘开始
      break;
    }
  }
  if(r_find)
  {
    for(i=r_find;i>round_sight;i--)
    {
      if(!Findline.rightlineflag[i])
      {
        r_lose=i;
        return 0;
      }
    }
    if(!r_lose) return 1;
    else return 0;
  }
  else return 0;
}

uint8 rround_identify2(void)
{
  int16 i;
  uint8 r_find=0,r_lose=0,r_findbefore=0;
  for(i=start_line-5;i>start_line-15;i--)
  {
    if(Findline.leftlineflag[i]) 
    {
      r_findbefore=1;
      break;
    }
  }
  for(i=start_line-10;i>round_sight;i--)
  {
    if(Findline.leftlineflag[i])
    {
      if((!r_findbefore&&Findline.leftline[i]<6)||r_findbefore) r_find=i;//开始丟线则找到线应从边缘开始
      break;
    }
  }
  if(r_find)
  {
    for(i=r_find;i>round_sight;i--)
    {
      if(!Findline.leftlineflag[i])
      {
        r_lose=i;
        return 0;
      }
    }
    if(!r_lose) return 1;
    else return 0;
  }
  else return 0;
}

//斜入圆环判断，先宽再窄再宽
uint8 round_identify3(void)
{
  int16 i;
  int16 l_wider_start=0,l_thiner_start=0,l_wider_end=0;
  for(i=start_line;i>Findline.endline;i--)
  {
    if(abs(Findline.rightline[i]-Findline.leftline[i])>(width[i]+5*WIDTH_ERR))
    {
      l_wider_start++;
    }
    else l_wider_start=0;
    if(l_wider_start>5)
    {
      l_wider_start=i;
      break;
    }
  }
  if(l_wider_start>80)
  {
    for(i=l_wider_start;i>Findline.endline;i--)
    {
      if(abs(Findline.rightline[i]-Findline.leftline[i])<(width[i]+WIDTH_ERR))
      {
        l_thiner_start=i;
        break;
      }
    }
    if(l_thiner_start)
    {
      for(i=start_line;i>Findline.endline;i--)
      {
        if(abs(Findline.rightline[i]-Findline.leftline[i])>(width[i]+5*WIDTH_ERR))
        {
          l_wider_end++;
        }
        else l_wider_end=0;
        if(l_wider_end>5)
        {
          l_wider_end=i;
          break;
        }
      }
      if(l_wider_end)
      {
        if(Findline.leftline[start_line-1]<midle_lineloc-width[i]/2+WIDTH_ERR&&Findline.leftline[start_line-2]<midle_lineloc-width[i]/2+WIDTH_ERR) road_type=ROUNDABOUTL;
        else road_type=ROUNDABOUTR;
        return 1;
      }
      else return 0;
    }
    else return 0;
  }
  else return 0;
}

void lround_check(void)
{
  int16 i;
  uint8 ln=0,lln=0;
  for(i=start_line-1;i>round_sight;i--)
  {
    if(!Findline.leftlineflag[i]) lln++;
    else ln++;
    if(ln&&!Findline.leftlineflag[i]) 
    {
      lln--;
      break;
    }
  }
  if(roundabout_type==0) 
  {
    roundabout_type=1;
    Findline.sight=straight_sight;
  }
  else if(roundabout_type==1&&((ln<2&&!lln)||lln>20)) roundabout_type=2;
  else if(roundabout_type==2&&ln>35&&!lln) 
  {
    roundabout_type=3;
    Findline.round_radius=ln;
//    before_locpicx=encoder.X;
//    before_locpicy=encoder.Y;
  }
  else if(roundabout_type==3&&ln<(Findline.round_radius*2/3)) 
  {
    roundstart_angle=gyro.TurnAngle_Integral;
    roundabout_type=4;
    before_locpicx=encoder.X;
    before_locpicy=encoder.Y;
  }
  else if(roundabout_type<6)//环内无照片
  {
    int16 leftlastpoint;
    leftlastpoint=100;
    for(i=start_line-1;i>10;i--)
    {
      if(Findline.leftlineflag[i]==0||Findline.leftline[i-1]<Findline.leftline[i])
      {
        leftlastpoint=i;
        break;
      }
    }
    if(leftlastpoint<60&&roundabout_type==4) roundabout_type=0;
  }
  else if(roundabout_type>5)//环内有照片
  {
    float angletemp=fabs(gyro.TurnAngle_Integral-roundstart_angle);
    if(roundabout_type==6) 
    {
      for(i=105;i>cur_sight+5;i--)//丟线
      {
        if(!Findline.rightlineflag[i])
        {
          roundabout_type=7;
          break;
        }
      }
    }
    else if(roundabout_type==7)
    {
      for(i=105;i>cur_sight;i--)//全不丟线
      {
        if(!Findline.rightlineflag[i])//&&Findline.rightline[Findline.endline]<midle_lineloc-20
        {
          roundabout_type=8;
          break;
        }
      }
      if(roundabout_type==7) 
      {
        roundabout_type=8;
      }
      else roundabout_type=7;
    }
    else if(roundabout_type==8)
    {
      for(i=105;i>cur_sight+5;i--)//该段全部丟线
      {
        if(Findline.rightlineflag[i])
        {
          roundabout_type=9;
          break;
        }
      }
      if(roundabout_type==8) roundabout_type=9;
      else roundabout_type=8;
    }
    else if(roundabout_type==9&&Findline.straightend<10&&(angletemp<10||angletemp>350)) roundabout_type=10;
    else if(roundabout_type==10) 
    {
      int16 leftlastpoint;
      leftlastpoint=0;
      for(i=start_line-1;i>10;i--)
      {
        if(Findline.leftlineflag[i]==0||Findline.leftline[i-1]>Findline.leftline[i])
        {
          leftlastpoint=i;
          break;
        }
      }
      if(leftlastpoint<70)
      {
        roundabout_type=0;
      }
    }
    else if((roundabout_type==6||roundabout_type==7)&&(angletemp>90)) roundabout_type=8;//强行推进状态
    else if((roundabout_type==8||roundabout_type==9)&&(angletemp<5||angletemp>355)) roundabout_type=10;//强行出圆环
  }
}

void rround_check(void)
{
  int16 i;
  uint8 rn=0,rln=0;
  for(i=start_line-1;i>round_sight;i--)
  {
    if(!Findline.rightlineflag[i]) rln++;
    else rn++;
    if(rn&&!Findline.rightlineflag[i]) 
    {
      rln--;
      break;
    }
  }
  if(roundabout_type==0) 
  {
    roundabout_type=1;
    Findline.sight=straight_sight;
  }
  else if(roundabout_type==1&&((!rln&&rn<2)||rln>20)) roundabout_type=2;//丟线
  else if(roundabout_type==2&&rn>35&&!rln)//不丟线
  {
    roundstart_angle=gyro.TurnAngle_Integral;
    roundabout_type=3;
    Findline.round_radius=rn;
//    before_locpicx=encoder.X;
//    before_locpicy=encoder.Y;
  }
  else if(roundabout_type==3&&rn<(Findline.round_radius*2/3))//环中间
  {
    roundabout_type=4;
    before_locpicx=encoder.X;
    before_locpicy=encoder.Y;
  }
  else if(roundabout_type<6)
  {
    int16 rightlastpoint;
    rightlastpoint=100;
    for(i=start_line-1;i>10;i--)
    {
      if(Findline.rightlineflag[i]==0||Findline.rightline[i-1]>Findline.rightline[i])
      {
        rightlastpoint=i;
        break;
      }
    }
    if(rightlastpoint<60&&roundabout_type==4) roundabout_type=0;//不丟线
  }
  else if(roundabout_type>5)//环内有照片
  {
    float angletemp=fabs(gyro.TurnAngle_Integral-roundstart_angle);
    if(roundabout_type==6) 
    {
      for(i=105;i>cur_sight+5;i--)//丟线
      {
        if(!Findline.leftlineflag[i])
        {
          roundabout_type=7;
          break;
        }
      }
    }
    else if(roundabout_type==7)
    {
      for(i=105;i>cur_sight;i--)//全不丟线
      {
        if(!Findline.leftlineflag[i])
        {
          roundabout_type=8;
          break;
        }
      }
      if(roundabout_type==7) roundabout_type=8;
      else roundabout_type=7;
    }
    else if(roundabout_type==8)
    {
      for(i=105;i>cur_sight+5;i--)//该段全部丟线
      {
        if(Findline.leftlineflag[i])
        {
          roundabout_type=9;
          break;
        }
      }
      if(roundabout_type==8) roundabout_type=9;
      else roundabout_type=8;
    }
    else if(roundabout_type==9&&Findline.straightend<10&&(angletemp<10||angletemp>350)) roundabout_type=10;
    else if(roundabout_type==10) 
    {
      int16 rightlastpoint;
      rightlastpoint=0;
      for(i=start_line-1;i>10;i--)
      {
        if(Findline.rightlineflag[i]==0||Findline.rightline[i-1]>Findline.rightline[i])
        {
          rightlastpoint=i;
          break;
        }
      }
      if(rightlastpoint<70)
      {
        roundabout_type=0;
      }
    }
    else if((roundabout_type==6||roundabout_type==7)&&(angletemp>90)) roundabout_type=8;//强行推进状态
    else if((roundabout_type==8||roundabout_type==9)&&(angletemp<5||angletemp>355)) roundabout_type=10;//强行出圆环
  }
}

void crossing_check(void)
{
  if(roundabout_type==0) 
  {
    roundabout_type=1;
  }
  else if(roundabout_type==1&&!Findline.rightlineflag[start_line-1]&&!Findline.leftlineflag[start_line-1]) roundabout_type=2;//先近距离丟线
  else if(roundabout_type==2&&Findline.rightlineflag[start_line-1]&&Findline.leftlineflag[start_line-1])//近距离不丟线
  {
    roundabout_type=3;
    before_locpicx=encoder.X;
    before_locpicy=encoder.Y;
  }
  else if(my_sqrt((encoder.X-before_locpicx)*(encoder.X-before_locpicx)+(encoder.Y-before_locpicy)*(encoder.Y-before_locpicy))>0.2&&roundabout_type==3)//(((abs(Findline.midline[Findline.endline]-midle_lineloc)>20&&(Findline.leftline[cur_sight-10]<Findline.leftline[cur_sight]||Findline.rightline[cur_sight-10]>Findline.rightline[cur_sight]))
//           ||Findline.straightend>20)&&roundabout_type==3)
  {
    roundabout_type=4;
    before_locpicx=encoder.X;
    before_locpicy=encoder.Y;
    if(Findline.midline[Findline.endline]<midle_lineloc) cross_task=1;
    else cross_task=2;
  }
//  else if(roundabout_type==4&&!Findline.rightlineflag[start_line-1]&&!Findline.leftlineflag[start_line-1]&&Findline.straightend<5)roundabout_type=5;//近距离丟线
//  else if(roundabout_type==5&&Findline.rightlineflag[start_line-1]&&Findline.leftlineflag[start_line-1])//近距离不丢线
//  {
//    roundabout_type=0;
//    cross_task=0;
//  }
  else
  {
    int16 i;
    uint8 lbreak_num=0,rbreak_num=0;
    uint8 left_all_w,right_all_w;
    left_all_w = 1;
    right_all_w = 1;
    for(i=cur_sight;i>Findline.straightend;i--)
    {
      if(Findline.rightlineflag[i]==0) lbreak_num++;
      else lbreak_num=0;
      if(lbreak_num>5) right_all_w=0;
      if(Findline.leftlineflag[i]==0) rbreak_num++;
      else rbreak_num=0;
      if(rbreak_num>5&&lbreak_num>5) 
      {
        left_all_w=0;
        right_all_w=0;
        break;
      }
    }
    if(right_all_w==0&&left_all_w==0) 
    {
      if(roundabout_type==4) roundabout_type=5;
      else if(roundabout_type==6) roundabout_type=7;
    }
    else if(left_all_w&&right_all_w&&(roundabout_type==5||roundabout_type==7)) 
    {
      roundabout_type=0;
      cross_task=0;
    }
  }
}

void jump_time(void)
{
  int16 i=0,j=0,j_time=0;
  uint8 line_time=0;
  if(startline_stop_flag==0) return;
  else
  {
    for(i=60;i>30;i--)
    {
      j_time=0;
      for(j=20;j<=160;j++)
      {
        if(mt9v03x_image_pross3[i][j]-mt9v03x_image_pross3[i][j+1]!=0||mt9v03x_image_pross3[i][j+1]-mt9v03x_image_pross3[i][j]!=0)//黑白跳变点
        {
                j_time++;
         }
      }
      if(j_time>=15)
      {
        line_time++;
      }
      if(line_time>=3)
      {
        startline_stop_flag=2;
        break;
      }
    }
  }
}

/*--------------------定位---------------------*/
void Find_pic(uint8 *p_pross)
{
  int16 i=0, j = 0, edge=0, pic_num;
  find_pic=0;
  curve_pic=0;
  int16 pic_sight,pic_end;
  pic_sight=pic_startline;
  pic_end=pic_sight-20;
//  if(road_type==STRAIGHT) 
//  {
//    pic_sight=pic_startline-10;
//    pic_end=pic_sight-30;
//  }
//  else 
//  {
//    pic_sight=pic_startline;
//    pic_end=pic_sight-20;
//  }
  if(Findline.leftline[pic_sight] < midle_lineloc) //先找左边图片
  {
    pic_num=0;
    for(i=pic_sight;i>pic_end;i--)
    {
      j = Findline.leftline[i];
      edge = Findline.leftline[i] < line_width +3 ? 3:Findline.leftline[i]-line_width;
      for(;j>edge;j--)
      {
        if(*(p_pross+i*MT9V03X_W+j-1) == 0 && *(p_pross+i*MT9V03X_W+j-2) == 255&&*(p_pross+i*MT9V03X_W+j-3) == 255&&Findline.leftlineflag[i])
        {
          pic_num++;
          break;
        }
      }
     if(pic_num>5)
     {
      find_pic=1;
      return;
      }
    }
  }
  if(Findline.rightline[pic_sight] > midle_lineloc&&find_pic==0&&Findline.rightlineflag[i])//找右边图片
  {
    pic_num=0;
    for(i=pic_sight;i>pic_end;i--)
    {
      j = Findline.rightline[i];
      edge = Findline.rightline[i] > 156 - line_width ? 156:Findline.rightline[i]+line_width;
      for(;j<edge;j++)
      {
        if(*(p_pross+i*MT9V03X_W+j+1) == 0 && *(p_pross+i*MT9V03X_W+j+2) == 255&& *(p_pross+i*MT9V03X_W+j+3) == 255&&Findline.rightlineflag[i])
        {
          pic_num++;
          break;
        }
      }
      if(pic_num>5)
      {
      find_pic=2;
      return;
      }
    }
  }
//  if(road_type==CURVE&&Findline.endline<15&&Findline.straightend<20)//大弯道，提前看
//  {
//    int16 pic_near_line=0,pic_endline=0,curve_start=0;
//    for(i=Findline.endline;i<start_line-1;i++)
//    {
//      if(abs(Findline.rightline[i]-Findline.leftline[i])>(width[i]+5*WIDTH_ERR))
//      {
//        curve_start=i;
//        break;
//      }
//    }
//    if(curve_start)
//    {
//      if(Findline.midline[Findline.endline]<midle_lineloc)//左弯
//      {
//        pic_num=0;     
//        for(j=Findline.leftline[curve_start];j>3;j--)
//        {
//          pic_near_line=Findline.endline+20>100?100:Findline.endline+20;
//          for(i=Findline.endline;i<pic_near_line;i++)//找弯道内边线
//          {
//            if(*(p_pross+(i-1)*MT9V03X_W+j-1)==255&&*(p_pross+i*MT9V03X_W+j-1)==255&&*(p_pross+(i+1)*MT9V03X_W+j-1)==0)
//            {
//              pic_near_line=i;
//              break;
//            }
//          }
//          if(pic_near_line<Findline.endline+20)//找到边线
//          {
//            pic_endline=pic_near_line+5>100?100:pic_near_line+5;
//            for(i=pic_near_line;i<pic_endline;i++)//找图片
//            {
//              if(*(p_pross+i*MT9V03X_W+j-1) == 0&&*(p_pross+(i+1)*MT9V03X_W+j-1)==255)
//              {
//                pic_num++;
//                break;
//              }
//            }
//            if(pic_num>10)
//            {
//              find_pic=1;
//              curve_pic=1;
//              return;
//            }
//          }
//        }
//        pic_num=0;
//        for(j=Findline.leftline[curve_start];j>3;j--)
//        {
//          pic_near_line=0;
//          for(i=Findline.endline;i>10;i--)//找上边线
//          {
//            if(*(p_pross+(i-1)*MT9V03X_W+j-1)==0&&*(p_pross+i*MT9V03X_W+j-1)==255&&*(p_pross+(i+1)*MT9V03X_W+j-1)==255)
//            {
//              pic_near_line=i;
//              break;
//            }
//          }
//          if(pic_near_line>10)//找到边线
//          {
//            pic_endline=pic_near_line-5<2?2:pic_near_line-5;
//            for(i=pic_near_line;i>pic_endline;i--)//找图片
//            {
//              if(*(p_pross+i*MT9V03X_W+j-1) == 0&&*(p_pross+(i-1)*MT9V03X_W+j-1) == 255)
//              {
//                pic_num++;
//                break;
//              }
//            }
//            if(pic_num>10)
//            {
//              find_pic=2;
//              curve_pic=2;
//              return;
//            }
//          }
//        }
//      }
//      else//右弯
//      {
//        pic_num=0;
//        for(j=Findline.rightline[curve_start];j<157;j++)
//        {
//          pic_near_line=Findline.endline+20>100?100:Findline.endline+20;
//          for(i=Findline.endline;i<pic_near_line;i++)//找弯道内边线
//          {
//            if(*(p_pross+(i-1)*MT9V03X_W+j+1)==255&&*(p_pross+i*MT9V03X_W+j+1)==255&&*(p_pross+(i+1)*MT9V03X_W+j+1)==0)
//            {
//              pic_near_line=i;
//              break;
//            }
//          }
//          if(pic_near_line<Findline.endline+20)//找到边线
//          {
//            pic_endline=pic_near_line+5>100?100:pic_near_line+15;
//            for(i=pic_near_line;i<pic_endline;i++)//找图片
//            {
//              if(*(p_pross+i*MT9V03X_W+j+1) == 0&&*(p_pross+(i+1)*MT9V03X_W+j+1) == 255)
//              {
//                pic_num++;
//                break;
//              }
//            }
//            if(pic_num>10)
//            {
//              find_pic=2;
//              curve_pic=1;
//              return;
//            }
//          }
//        }
//        pic_num=0;
//        for(j=Findline.rightline[curve_start];j<157;j++)
//        {
//          pic_near_line=0;
//          for(i=Findline.endline;i>10;i--)//找上边线
//          {
//            if(*(p_pross+(i-1)*MT9V03X_W+j+1)==0&&*(p_pross+i*MT9V03X_W+j+1)==255&&*(p_pross+(i+1)*MT9V03X_W+j+1)==255)
//            {
//              pic_near_line=i;
//              break;
//            }
//          }
//          if(pic_near_line>10)//找到边线
//          {
//            pic_endline=pic_near_line-5<2?2:pic_near_line-15;
//            for(i=pic_near_line;i>pic_endline;i--)//找图片
//            {
//              if(*(p_pross+(i-1)*MT9V03X_W+j+1) == 255&&*(p_pross+i*MT9V03X_W+j+1) == 0)
//              {
//                pic_num++;
//                break;
//              }
//            }
//            if(pic_num>10)
//            {
//              find_pic=1;
//              curve_pic=2;
//              return;
//            }
//          }
//        }
//      }
//    }
//  }
}

//void find_pic_none(uint8 *p_pross)
//{
//  int16 i,j;
//  uint16 white_num;
//  white_num=0;
//  for(i=110;i>70;i--)
//  {
//    for(j=60;j<100;j++)
//    {
//      if(*(p_pross+i*MT9V03X_W+j)==255) white_num++;
//      if(white_num>300)
//      {
//        j=100;i=70;
//      }
//    }
//  }
//  if(white_num>300)
//  {
//    imageProcess_found=1;
//    return;
//  }
//  else
//  {
//    imageProcess_found=0;
//    return;
//  }
//}
//
//void pic_locate(uint8 *p_pross)
//{
//  int16 i,j,pic_up,pic_left,pic_right,pic_mid;
//  pic_right=0;
//  // 0   0   0   0   0
//  // 0  255 255 255  0  (real_pointy)
//  for(i=50;i<start_line-5;i++)
//  {
//    pic_up=0;
//    pic_mid=0;
//    for(j=20;j<140;j++)
//    {
//      if(*(p_pross+(i+2)*MT9V03X_W+j)==255&&*(p_pross+(i+1)*MT9V03X_W+j)==255 && *(p_pross+i*MT9V03X_W+j)==0)
//      {
//        pic_up++;
//        pic_right=j;
//        if(pic_mid==0) pic_mid=j+5;
//      }
//    }
//    if(pic_up>5&&pic_up<40&&pic_right<130)//防止噪点和赛道的干扰
//    {
//      pic_up=i;
//      real_pointy=pic_up;
//      break;
//    }
//    else pic_up=0;
//  }
//  if(pic_up)
//  {
//    pic_up=real_pointy+10;
//    imageProcess_found=0;
//    pic_left=0;
//    pic_right=0;
//   //       255
//  //        255
//  //0  0  *(pic_left)   
//  //        255
//  //        255
//    for(i=pic_mid;i>20;i--)
//    {
//      if(*(p_pross+pic_up*MT9V03X_W+i-2)==0&&*(p_pross+pic_up*MT9V03X_W+i-1)==0&&*(p_pross+(pic_up-1)*MT9V03X_W+i+1)==255&&*(p_pross+(pic_up-2)*MT9V03X_W+i+1)==255&&*(p_pross+(pic_up+1)*MT9V03X_W+i+1)==255&&*(p_pross+(pic_up+2)*MT9V03X_W+i+1)==255)
//      {
//        pic_left=i;
//        break;
//      }
//    }
//    //            255
//    //            255
//    //       (pic_right)*  0  0
//    //            255
//    //            255
//    for(i=pic_mid;i<140;i++)
//    {
//      if(*(p_pross+pic_up*MT9V03X_W+i+2)==0&&*(p_pross+pic_up*MT9V03X_W+i+1)==0&&*(p_pross+(pic_up-1)*MT9V03X_W+i-1)==255&&*(p_pross+(pic_up-2)*MT9V03X_W+i-1)==255&&*(p_pross+(pic_up+1)*MT9V03X_W+i-1)==255&&*(p_pross+(pic_up+2)*MT9V03X_W+i-1)==255)
//      {
//        pic_right=i;
//        break;
//      }
//    }
//    if(pic_left<pic_right&&(pic_right-pic_left)<40) 
//    {
//      real_pointx=(pic_left+pic_right)/2;
//      float dxb=(float)(real_pointx-MTXL_EXP)*0.0001*15;
//      float dyb=(float)(MTYL_EXP-real_pointy)*0.0001*15;
//      if(abs(MTXL_EXP-real_pointx)<4 && abs(real_pointy-MTYL_EXP)<4)
//      {
//        imageProcessFinish_flag=1;
//        dxb=0;
//        dyb=0;
//      }
//      SetX = encoder.X + dyb*sin(PI*gyro.TurnAngle_Integral/180) + dxb*cos(PI*gyro.TurnAngle_Integral/180);
//      SetY = encoder.Y + dyb*cos(PI*gyro.TurnAngle_Integral/180) - dxb*sin(PI*gyro.TurnAngle_Integral/180);
//      real_pointx=MTXL_EXP;
//      real_pointy=MTYL_EXP;
//    }
//    else
//    {
//      imageProcessFinish_flag=1;
//    }
//  } 
//  else
//  {
//    if(imageProcess_found)
//    {
//      imageProcessFinish_flag=1;
//    }
//    else
//    {
//      imageProcess_found=1;
////      float dyb= -0.05;
////      SetX = encoder.X + dyb*sin(PI*gyro.TurnAngle_Integral/180);
////      SetY = encoder.Y + dyb*cos(PI*gyro.TurnAngle_Integral/180);
//    }
//  }
//}
//
//void pic_locate2(uint8 *p_pross)
//{
//  int16 i;
//  int16 pic_down=0,pic_left=0,pic_right=155;
//  for(i=start_line-10;i>30;i--)
//  {
//    if(*(p_pross+(i+2)*MT9V03X_W+midle_lineloc)==0&&*(p_pross+(i+1)*MT9V03X_W+midle_lineloc)==0 && *(p_pross+i*MT9V03X_W+midle_lineloc)==255)
//    {
//      pic_down=i;
//      real_pointy=i;
//      break;
//    }
//  }
//  if(pic_down)
//  {
//    pic_down=real_pointy-10;
//   //       255
//  //        255
//  //0  0  *(pic_left)   
//  //        255
//  //        255
//    for(i=midle_lineloc;i>20;i--)
//    {
//      if(*(p_pross+pic_down*MT9V03X_W+i-2)==0&&*(p_pross+pic_down*MT9V03X_W+i-1)==0&&*(p_pross+(pic_down-1)*MT9V03X_W+i+1)==255&&*(p_pross+(pic_down-2)*MT9V03X_W+i+1)==255&&*(p_pross+(pic_down+1)*MT9V03X_W+i+1)==255&&*(p_pross+(pic_down+2)*MT9V03X_W+i+1)==255)
//      {
//        pic_left=i;
//        break;
//      }
//    }
//    //            255
//    //            255
//    //       (pic_right)*  0  0
//    //            255
//    //            255
//    for(i=midle_lineloc;i<140;i++)
//    {
//      if(*(p_pross+pic_down*MT9V03X_W+i+2)==0&&*(p_pross+pic_down*MT9V03X_W+i+1)==0&&*(p_pross+(pic_down-1)*MT9V03X_W+i-1)==255&&*(p_pross+(pic_down-2)*MT9V03X_W+i-1)==255&&*(p_pross+(pic_down+1)*MT9V03X_W+i-1)==255&&*(p_pross+(pic_down+2)*MT9V03X_W+i-1)==255)
//      {
//        pic_right=i;
//        break;
//      }
//    }
//    real_pointx=(pic_left+pic_right)/2;
//    float dxb=(float)(real_pointx-MTXL_EXP)*0.0001*15;
//    float dyb=(float)(MTYL_EXP+15-real_pointy)*0.0001*15;
//    if(abs(MTXL_EXP-real_pointx)<10 && abs(real_pointy-MTYL_EXP)<10)
//    {
//      imageProcessFinish_flag=1;
//      dxb=0;
//      dyb=0;
//    }
//    SetX = encoder.X + dyb*sin(PI*gyro.TurnAngle_Integral/180) + dxb*cos(PI*gyro.TurnAngle_Integral/180);
//    SetY = encoder.Y + dyb*cos(PI*gyro.TurnAngle_Integral/180) - dxb*sin(PI*gyro.TurnAngle_Integral/180);
//    real_pointx=MTXL_EXP;
//    real_pointy=MTYL_EXP+15;
//  }
//  else 
//  {
//    imageProcess_found=1;
//    imageProcessFinish_flag=1;
//  }
//}
//
//
//int16 blob_information[3][2]={{0,160},{0,0},{0,0}};
//void pic_locate_angle(uint8 *p_pross)
//{
//  blob_information[0][0]=0;
//  blob_information[0][1]=160;
//  blob_information[1][0]=0;
//  blob_information[2][1]=0;
//  int16 i,j;
//  uint8 road_num,scan_start=0;
////  int16 blob_information[3][2]={{0,160},{0,0},{0,0}};
//  for(j=10;j<20;j++)
//  {
//    for(i=90;i>50;i--)
//    {
//      if(*(p_pross+i*MT9V03X_W+j)==0) 
//      {
//        road_num++;
//        break;
//      }
//    }
//    if(road_num>15) 
//    {
//      scan_start=1;
//      break;
//    }
//    else scan_start=2;
//  }
//  if(scan_start==2)//视野左边有赛道，从右往左找
//  {
//    road_num=0;
//    for(j=140;j>20;j--)
//    {
//      road_num++;
//      for(i=start_line-2;i>70;i--)
//      {
//        if(*(p_pross+i*MT9V03X_W+j)==255&&*(p_pross+(i+1)*MT9V03X_W+j)==0&&*(p_pross+(i+2)*MT9V03X_W+j)==0)
//        {
//          road_num=0;
//          if(j<blob_information[0][1])//左边顶点 
//          {
//            blob_information[0][0]=i;
//            blob_information[0][1]=j;
//          }
//          if(i>blob_information[1][0])//下边顶点
//          {
//            blob_information[1][0]=i;
//            blob_information[1][1]=j;
//          }
//          if(j>blob_information[2][1])//右边顶点
//          {
//            blob_information[2][0]=i;
//            blob_information[2][1]=j;
//          }
//          break;
//        }
//      }
//      if(blob_information[0][0]&&road_num>10) break;
//    }
//  }
//  else//视野左边无赛道，从左开始扫描
//  {
//    road_num=0;
//    for(j=20;j<140;j++)
//    {
//      road_num++;
//      for(i=start_line-2;i>70;i--)
//      {
//        if(*(p_pross+i*MT9V03X_W+j)==255&&*(p_pross+(i+1)*MT9V03X_W+j)==0&&*(p_pross+(i+2)*MT9V03X_W+j)==0)
//        {
//          road_num=0;
//          if(j<blob_information[0][1]) 
//          {
//            blob_information[0][0]=i;
//            blob_information[0][1]=j;
//          }
//          if(i>blob_information[1][0])
//          {
//            blob_information[1][0]=i;
//            blob_information[1][1]=j;
//          }
//          if(j>blob_information[2][1])
//          {
//            blob_information[2][0]=i;
//            blob_information[2][1]=j;
//          }
//          break;
//        }
//      }
//      if(blob_information[0][0]!=0&&(road_num>2)) break;
//    }
//  }
//  if(blob_information[0][0]&&blob_information[1][1]&&blob_information[2][0])//要看到完整图片
//  {
//    float pic_angle=0;
//    if(blob_information[2][0]>blob_information[0][0])//图片左倾
//    {
//      pic_angle=(float)(blob_information[1][0]-blob_information[2][0]);
//      if(fabs(pic_angle)<3)
//      {
//        angle_correct=1;
//      }
//      else
//      {
//        pic_angle=pic_angle/(float)(blob_information[2][1]-blob_information[1][1]);
//        pic_angle=atan(pic_angle)*180/PI;
//        angleSet-=pic_angle;
//        float distance_pic=0;
//        distance_pic=(float)((MTYL_EXP-10)*2-blob_information[2][0]-blob_information[0][0])/200;
//        float dxb,dyb;
//        dxb=distance_pic*sin(pic_angle);
//        dyb=distance_pic*(1-cos(pic_angle));
//        SetX = encoder.X + dyb*sin(PI*gyro.TurnAngle_Integral/180) + dxb*cos(PI*gyro.TurnAngle_Integral/180);
//        SetY = encoder.Y + dyb*cos(PI*gyro.TurnAngle_Integral/180) - dxb*sin(PI*gyro.TurnAngle_Integral/180);
//      }
//    }
//    else//图片右倾
//    {
//      pic_angle=(float)(blob_information[1][0]-blob_information[0][0]);
//      if(fabs(pic_angle)<3)
//      {
//        angle_correct=1;
//        //real_pointy=(uint16)(pic_angle/2+0.5);
//      }
//      else
//      {
//        pic_angle=pic_angle/(float)(blob_information[1][1]-blob_information[0][1]);
//        pic_angle=atan(pic_angle)*180/PI;
//        angleSet+=pic_angle;
//        float distance_pic=0;
//        distance_pic=(float)((MTYL_EXP-10)*2-blob_information[1][0]-blob_information[0][0])/200;
//        float dxb,dyb;
//        dxb=-distance_pic*sin(pic_angle);
//        dyb=distance_pic*(1-cos(pic_angle));
//        SetX = encoder.X + dyb*sin(PI*gyro.TurnAngle_Integral/180) + dxb*cos(PI*gyro.TurnAngle_Integral/180);
//        SetY = encoder.Y + dyb*cos(PI*gyro.TurnAngle_Integral/180) - dxb*sin(PI*gyro.TurnAngle_Integral/180);
//      }
//    }
//  }
//  else
//  {
//    if(imageProcess_found)
//    {
//      imageProcessFinish_flag=1;
//      angle_correct=0;
//      imageProcess_found=0;
//    }
//    else
//    {
//      imageProcess_found=1;
//      float dyb= -0.001*40;
//      SetX = encoder.X + dyb*sin(PI*gyro.TurnAngle_Integral/180);
//      SetY = encoder.Y + dyb*cos(PI*gyro.TurnAngle_Integral/180);
//    }
//  }
//}
/*--------------------快速排序---------------------*/
int findPos(uint8 data[], int low, int high) {
    //将大于t的元素赶到t的左边，小于t的元素赶到t的右边
    uint8 t = data[low];
    while(low < high) {
        while(low < high && data[high] >= t) {
            high--;
        }
        data[low] = data[high];
        while(low < high && data[low] <=t) {
            low++;
        }
        data[high] = data[low];
    }
    data[low] = t;
    //返回此时t在数组中的位置
    return low;
}
//在数组中找一个元素，对大于该元素和小于该元素的两个数组进行再排序
//再对两个数组分为4个数组，再排序，直到最后每组只剩下一个元素为止
void quickSort(uint8 data[], int low, int high) {
    if(low > high) {
        return;
    }
    int pos = findPos(data, low, high);
    quickSort(data, low, pos-1);
    quickSort(data, pos+1, high); 
}

#define RATIOMAX        1.4
#define RATIOMIN        0.3
#define XMAX            MT9V03X_W/5*4
#define YMAX            MT9V03X_H/5*4


/**********************************************************************************************
@brief          滤除二值化图像中的独立噪点（或许没啥用处~~）
@param          uint8 *pic              源图片数据
@param          uint8 *processed_pic    处理后的图片数据     	        
@return         void
Sample usage:   FilterIndependentNoise(mt9v03x_csi_image_pross1[0],mt9v03x_csi_image_pross1plus[0]);
**********************************************************************************************/
void FilterIndependentNoise(uint8 *pic,uint8 *processed_pic)
{
  uint16 i,j;
  for(j=1;j<MT9V03X_H-1;j++)
  {
    for(i=1;i<MT9V03X_W-1;i++)
    {
      if((*(pic+j*MT9V03X_W+i) == 255)
         &&(                                    (*(pic+j*MT9V03X_W+i+1))      +     (*(pic+j*MT9V03X_W+i-1))
           +(*(pic+(j+1)*MT9V03X_W+i))+     (*(pic+(j+1)*MT9V03X_W+i+1))  +     (*(pic+(j+1)*MT9V03X_W+i-1))
           +(*(pic+(j-1)*MT9V03X_W+i))+     (*(pic+(j-1)*MT9V03X_W+i+1))  +     (*(pic+(j-1)*MT9V03X_W+i-1))
           ) < 600)     {*(processed_pic+j*MT9V03X_W+i) = 0;}

      else if((*(pic+j*MT9V03X_W+i) == 0)
         &&(                                    (*(pic+j*MT9V03X_W+i+1))      +     (*(pic+j*MT9V03X_W+i-1))
           +(*(pic+(j+1)*MT9V03X_W+i))+     (*(pic+(j+1)*MT9V03X_W+i+1))  +     (*(pic+(j+1)*MT9V03X_W+i-1))
           +(*(pic+(j-1)*MT9V03X_W+i))+     (*(pic+(j-1)*MT9V03X_W+i+1))  +     (*(pic+(j-1)*MT9V03X_W+i-1))
           ) > 1500)    {*(processed_pic+j*MT9V03X_W+i) = 255;} 
      else
                        {*(processed_pic+j*MT9V03X_W+i) = *(pic+j*MT9V03X_W+i);}
    }
   }
}

/**********************************************************************************************
@brief         最朴实无华的二值化，不损失图像
@param          uint8 *p                源图片数据
@param          uint8 *p_pross          处理后的图片数据     	        
@return         void
Sample usage:  
***************************************************************************************/
void binaryzationOperate_simple(uint8 *p,uint8 *p_pross,uint8 *p_pic)
{ 
  uint16 i,j;
  for(j=0;j<MT9V03X_H;j++)
  {
    for(i=0;i<MT9V03X_W;i++)
    {
      if(*(p+j*MT9V03X_W+i)>THRESHOLD)
      {
        *(p_pross+j*MT9V03X_W+i) = 255;
      }
      else 
      {
        *(p_pross+j*MT9V03X_W+i) = 0;
      }
      if(*(p+j*MT9V03X_W+i)>THRESHOLD_PIC)
      {
        *(p_pic+j*MT9V03X_W+i) = 255;
      }
      else
      {
        *(p_pic+j*MT9V03X_W+i) = 0;
      }
    }
  }    
}

void display_line(uint8 *p_pross)
{
  int16 i;
  for(i=start_line;i>Findline.endline;i--)
  {
    *(p_pross+i*MT9V03X_W+Findline.midline[i]) = 0;
  } 
  for(i=start_line;i>Findline.straightend;i--)
  {
    *(p_pross+i*MT9V03X_W+midle_lineloc) = 0;
  }
}


