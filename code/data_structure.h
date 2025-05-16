#ifndef _DATA_STRUCTURE
#define _DATA_STRUCTURE
#include "zf_common_headfile.h"

#define MAX_SIZE_STACK 512
#define OK 1
#define ERROR 0
#define TRUE 1
#define FALSE 0
#define MAXSIZE MT9V03X_W*MT9V03X_H

typedef uint16 ElemType;
typedef uint8 State;
typedef struct
{   //˳��ջ�Ĵ洢�ṹ
    ElemType * data ;//�����������ݣ����ΪMAXSIZE����Ϊջ������
    int top;   //����ջ��ָ��
}SqStack;

State initStack(SqStack *S);
int getLength(SqStack S);
State clearStack(SqStack *S);
State isEmpty(SqStack *S);
State push(SqStack *S, ElemType e);
State pop(SqStack *S, ElemType *e);
State getTop(SqStack S, ElemType *e);


#endif








