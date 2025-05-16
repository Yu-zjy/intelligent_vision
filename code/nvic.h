#ifndef __NVIC_H
#define __NVIC_H
#include "zf_common_headfile.h"

#define NVIC_PriorityGroup_0 ((uint32_t)0x07)
#define NVIC_PriorityGroup_1 ((uint32_t)0x06)
#define NVIC_PriorityGroup_2 ((uint32_t)0x05)
#define NVIC_PriorityGroup_3 ((uint32_t)0x04)
#define NVIC_PriorityGroup_4 ((uint32_t)0x03)

#define IS_NVIC_PRIORITY_GROUP(GROUP) (((GROUP) == NVIC_PriorityGroup_0) || \
                                       ((GROUP) == NVIC_PriorityGroup_1) || \
                                       ((GROUP) == NVIC_PriorityGroup_2) || \
                                       ((GROUP) == NVIC_PriorityGroup_3) || \
                                       ((GROUP) == NVIC_PriorityGroup_4))

#define IS_NVIC_PREEMPTION_PRIORITY(PRIORITY)  ((PRIORITY) < 0x10)

#define IS_NVIC_SUB_PRIORITY(PRIORITY)  ((PRIORITY) < 0x10)

//typedef struct
//{
//	IRQn_Type NVIC_IRQChannel;
//	uint32 NVIC_IRQChannelGroupPriority;
//	uint32 NVIC_IRQChannelPreemptionPriority;
//	uint32 NVIC_IRQChannelSubPriority;
//	uint8 NVIC_IRQChannelEnable;
//} NVIC_InitTypeDef;
//
//uint8 NVIC_Init(NVIC_InitTypeDef);
void Priority_Init(void);

#endif