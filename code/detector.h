#ifndef _DETECTOR_H
#define _DETECTOR_H

#include "zf_common_headfile.h"

#define DETECTOR_PIN1 B15
#define DETECTOR_PIN2 B14

extern rt_sem_t detector_sem;
void detector_init(void);

#endif