#include "FreeRTOSTasks.h"
#include "mpu6050.h"
#include "usart.h"
#include "retarget.h"
#include "angle.h"
#include "math.h"


TaskHandle_t handleTaskDisplay;
void TaskDisplay(void *pvParameters);




void InitTasks(void)//创建任务
{
    xTaskCreate(TaskDisplay,
                "TaskDisplay",
                1000,
                NULL,
                1,
                &handleTaskDisplay);//屏幕刷新线程
}

void TaskDisplay(void *pvParameters)
{
    while(1)
    {
        OLED_ShowNum(0,0,0,1,1);
    }
}