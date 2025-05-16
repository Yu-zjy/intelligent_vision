#ifndef _IMAGE
#define _IMAGE
#include "zf_common_headfile.h"
#include "math.h"

#define LINECOLOR  RGB565_RED
//由于屏幕的缩放，屏幕的像素坐标不与图像坐标对应 
//将图像的坐标 （x,y）映射到屏幕上
#define STANDPOINTX(x) (uint16)(x*ips114_x_max/MT9V03X_W)
#define STANDPOINTY(y) (uint16)(y*ips114_y_max/MT9V03X_H)

#define imgw                    MT9V03X_W-4


#define left_garage 0
#define right_garage 1
#define straight_sight  30
#define cur_sight       45
#define round_sight     15
#define line_width      10
#define midle_lineloc   82
#define start_line      108 //图像处理起始行
#define pic_startline   90//图片处理起始行
#define WIDTH_ERR       2
#define CAM_REA         200
#define ROUND_MASK      100
#define STRAIGHT        0
#define CURVE           1 
#define ROUNDABOUTL     2
#define ROUNDABOUTR     3
#define CROSSING        4
#define BLOCK           5
#define LOSE            6

#define THRESHOLDFORPICTURE  90
#define THRESHOLDFORBACK     120

extern uint8 mt9v03x_binary_finish_flag; 
extern uint8 mt9v03x_pross_finish_flag;               //一场图像处理完成标志位
extern uint8 (*mt9v03x_image_pross1)[MT9V03X_W];          //图像处理后数据
extern uint8 (*mt9v03x_image_pross2)[MT9V03X_W];          //图像处理后数据
extern uint8 (*mt9v03x_image_pross3)[MT9V03X_W];          //图像处理后数据
extern uint8 (*mt9v03x_image_pross4)[MT9V03X_W];          //图像处理后数据
extern uint8 THRESHOLD;
extern uint8 find_pic,curve_pic;

typedef struct  __findline_struct
{
    int16 upline[160];
    int16 midline[120];
    int16 leftline[120];
    int16 rightline[120];
    int16 leftlineflag[120];
    int16 rightlineflag[120];
//    int16 line_with_test[120];
    int16 leftstartpoint;
    int16 rightstartpoint;
    int16 endline;
    int16 straightend;
    uint8 round_radius;
    int8 loseflag;
    int16 sight;
    int filter[3];
    float err[5];
    float angle_err;//角度偏差
    float angle_last_err;
    float pic_road_angle;
    int16 ljumppoint;
    int16 rjumppoint;
} findline_TypeDef;

uint8 otsuThreshold(uint8 *image);
void image_init(void);
void binaryzationOperate_simple(uint8 *p,uint8 *p_pross,uint8 *p_pic);




#endif