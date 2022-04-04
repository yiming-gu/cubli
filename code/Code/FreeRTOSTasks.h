#ifndef FREERTOSTASKS_H
#define FREERTOSTASKS_H

#include "FreeRTOS.h"
#include "task.h"
#include "oled.h"

void InitTasks(void);
void TaskDisplay(void *pvParameters);




#endif