#include "data_structure.h"

//static rt_uint8_t mempool[1024];
//static struct rt_mempool mp;
//
//void dataStructure_memint(void)
//{
//  /* 初始化内存池对象 */
//  rt_mp_init(&mp, "mp1", &mempool[0], sizeof(mempool), 1020);//在初始化内存池对象时，初始化了 4096 /(80+4) = 48 个内存块。
//}
/*****************链式栈*****************************/ 


AT_DTCM_SECTION_ALIGN(ElemType data_storePoint [MAXSIZE], 64);


//初始化顺序栈
//目前只支持使用一个栈
State initStack(SqStack *S){
    S->top = -1;   //将栈顶指针置为-1，即将栈作为还是空的时候
    S->data= data_storePoint;
    return OK;
}

//获得顺序栈的长度
int getLength(SqStack S){
    return (S.top)+1;   //根据数组下标的规则，数组中的长度为指针+1个元素
}

//清空顺序栈
State clearStack(SqStack *S){
  if(S->top != -1)
    for(int i= 0; i<=S->top;i++)
      S->data[i]=0;
  S->top = -1;   //将栈顶指针重新设置为-1，即此时表示栈空
  return OK;
}

//判断顺序栈是否为空
State isEmpty(SqStack *S){
    if(S->top==-1){   //如果此时栈顶指针为-1表示栈此时为空，非-1则表示非空
        return TRUE;
    }else{
        return FALSE;
    }
}

//入栈
State push(SqStack *S, ElemType e){
    if(S->top==MAXSIZE-1){   //根据数组下标的特点，当指针指向最后一个元素时为MAXSIZE-1，此时栈满
        //rt_kprintf("stack is full...\n");
        return ERROR;   //栈满说明空间已满已经不可以再入栈
    }else{    //如果栈非满则执行添加过程
        S->top++;   //栈顶指针+1指向一个新的顶部空间
        S->data[S->top]=e;   //将现在指向的这个新的空的栈顶空间元素置为指定元素（后进先出）
        return OK;
    }
}

//出栈
State pop(SqStack *S, ElemType *e){
    if(S->top==-1){   //当栈顶指针指向-1，说明栈空，则无法出栈
        //rt_kprintf("stack is full...ERROR to pop\n");
        return ERROR;
    }else{   //如果栈非空则执行出栈程序
        *e = S->data[S->top];   //将当前栈顶元素的指针赋给可供返回查看的e
        S->top--;   //栈顶元素出栈后，栈顶指针向下走一格，表示新的栈顶元素
        return OK;
    }
}

//获取栈顶元素（只供查看，不出栈）
State getTop(SqStack S, ElemType *e){
    if(S.top==-1){   //当栈顶指针指向-1，说明栈空，栈顶元素为空
        rt_kprintf("stack is empty...ERROR to getpop\n");
        return ERROR;
    }else{   //当栈非空的时候，则将栈顶元素赋值给可供返回查看的e，但是栈顶元素并不出栈
        *e = S.data[S.top];   //将栈顶元素赋值给e，栈顶指针top不变
        return OK;
    }
}

//遍历打印顺序栈
//State printStack(SqStack S){
//    if(S.top==-1){   //当栈顶指针指向-1，说明栈空，无栈元素可供打印
//        rt_kprintf("stack is empty...ERROR to getpop\n");
//        return ERROR;
//    }
//    int i=0;   //计数器，记录当前是第几个元素
//    while(S.top!=-1){
//        i++;   //栈顶指针还未到-1，则说明当前栈顶指针有元素，计数器+1
//        printf("栈顶向下第%d个元素为：%d\n", i, S.data[S.top]);  //当前栈顶指针的元素打印出
//        S.top--;   //栈顶指针向下走一格，继续进行循环打印
//    }
//    return OK;
//}