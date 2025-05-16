#ifndef _POSITION_H
#define _POSITION_H
#include "zf_common_headfile.h"

#define MAXYSPEED               1.5

#define ANGLESPEED              1.5

typedef enum //车辆状态定义
{
  START,//起点
  FINDLINE,//巡线
  APPROACH_BOX,   // 接近箱子
  ADJUST_POSE,    // 调整姿态
  PUSH_LEFT,      // 向左推箱
  PUSH_RIGHT,     // 向右推箱
  RETURN_TRACK,    // 返回赛道
  MOVE_RIGHT,         // 向右横向移动（左推准备）
  MOVE_LEFT,          // 向左横向移动（右推准备）
  MOVE_FORWARD_PUSH,  // 纵向靠近箱子
  PUSH_LEFT_STEP,     // 左推执行阶段
  PUSH_RIGHT_STEP,     // 右推执行阶段  
  FACEPIC,//面向图片
  FINDPIC,//定位图片
  TARGET_IDENTIFY,//目标图片识别
  TARGET_CARRY,//拾取图片
  CORRECT_PIC,//确定图片搬完与否
  FINDROUNDPIC,//环岛定位图片
  FINDAGAIN_PIC,//环岛再次确认图片
  ROUND_BACK,//二次确认环岛后退回
  CROSSING_FIRST,//十字第一个框
  ROUND_LINE,//元素巡线
  CLASSIFY_LOC,//元素分类框定位
  ELEMENT_IDENTIFY,//元素分类
  TARGET_LAY,//放下图片
  ELEMENTBACK,//定位字母后回赛道
  BACKFORSTART,
  START_FINDRECT,//起点处归置时巡线
  START_FINDRECTL,
  START_FINDRECTR,
  START_IDENTIFY,//起点数字识别
  START_LAY,//起点归置
  START_BACK,//回赛道
  STOPFOREND,//任务完成
  DEBUG_STATUS,//调试时使用
}CARSTATUS_enum;

typedef struct//用以存储目标物的坐标
{
    float x;
    float y;
    uint8 flag;//该目标是否还未处理 1 未处理  0 已处理
}targetPosition;
extern uint8 IsPosition_get;
extern float xSet1,ySet1,xSet2,ySet2,xSet3,ySet3,xSet4,ySet4,xSet5,ySet5;
extern uint8 startCarFlag;
extern int terify_flag;
#endif
