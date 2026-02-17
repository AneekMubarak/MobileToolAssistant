#include <stdio.h>
#include <unistd.h>
#include "rtos.h"
#include "motor_hw.h"

QueueHandle_t xMotorQueue;

int main()
{
    // Init Motor Command Queue
    xMotorQueue = xQueueCreate(1, sizeof(RobotAction_t));

    if (xMotorQueue == NULL)
    {
        // Queue creation failed (not enough heap)
        while(1);
    }

    while(1)
    {
        Motor_Task();
        sleep(1);
        ObstacleDet_Task();
        sleep(1);
        Motor_Task();
        sleep(1);
    }

    return 0;
}