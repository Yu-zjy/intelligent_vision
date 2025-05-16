#ifndef _POSITION_H
#define _POSITION_H
#include "zf_common_headfile.h"

#define MAXYSPEED               1.5

#define ANGLESPEED              1.5

typedef enum //����״̬����
{
  START,//���
  FINDLINE,//Ѳ��
  APPROACH_BOX,   // �ӽ�����
  ADJUST_POSE,    // ������̬
  PUSH_LEFT,      // ��������
  PUSH_RIGHT,     // ��������
  RETURN_TRACK,    // ��������
  MOVE_RIGHT,         // ���Һ����ƶ�������׼����
  MOVE_LEFT,          // ��������ƶ�������׼����
  MOVE_FORWARD_PUSH,  // ���򿿽�����
  PUSH_LEFT_STEP,     // ����ִ�н׶�
  PUSH_RIGHT_STEP,     // ����ִ�н׶�  
  FACEPIC,//����ͼƬ
  FINDPIC,//��λͼƬ
  TARGET_IDENTIFY,//Ŀ��ͼƬʶ��
  TARGET_CARRY,//ʰȡͼƬ
  CORRECT_PIC,//ȷ��ͼƬ�������
  FINDROUNDPIC,//������λͼƬ
  FINDAGAIN_PIC,//�����ٴ�ȷ��ͼƬ
  ROUND_BACK,//����ȷ�ϻ������˻�
  CROSSING_FIRST,//ʮ�ֵ�һ����
  ROUND_LINE,//Ԫ��Ѳ��
  CLASSIFY_LOC,//Ԫ�ط����λ
  ELEMENT_IDENTIFY,//Ԫ�ط���
  TARGET_LAY,//����ͼƬ
  ELEMENTBACK,//��λ��ĸ�������
  BACKFORSTART,
  START_FINDRECT,//��㴦����ʱѲ��
  START_FINDRECTL,
  START_FINDRECTR,
  START_IDENTIFY,//�������ʶ��
  START_LAY,//������
  START_BACK,//������
  STOPFOREND,//�������
  DEBUG_STATUS,//����ʱʹ��
}CARSTATUS_enum;

typedef struct//���Դ洢Ŀ���������
{
    float x;
    float y;
    uint8 flag;//��Ŀ���Ƿ�δ���� 1 δ����  0 �Ѵ���
}targetPosition;
extern uint8 IsPosition_get;
extern float xSet1,ySet1,xSet2,ySet2,xSet3,ySet3,xSet4,ySet4,xSet5,ySet5;
extern uint8 startCarFlag;
extern int terify_flag;
#endif
