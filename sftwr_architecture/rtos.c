#include <stdio.h>
#include "rtos.h"
#include "motor_hw.h"
#include "ObstDet_hw.h"



// Var Definitions
#define BASE_SPEED 60


// Task Definitions
void Motor_Task(void)
{
    RobotAction_t receivedAction;

    for(;;) {
        // Low Prio
        printf("Motor running logic\n");

        printf("Getting motor commmands...\n");
        // Get Motor command and store in receivedAction
        xQueueReceive(xMotorQueue, &receivedAction, portMAX_DELAY); 


        printf("Driving Motors...\n");
        Motor_ExecuteAction(receivedAction); // Defined and done my motor_hw.c

    }
    
}

void ObstacleDet_Task(void)
{
    ToF_Data_t sensorData;
    RobotAction_t DodgeAction;

    ObstDet_HW_Init();
    Obstacle_Init();

    for(;;){
        // Mid Prio
        printf("Obstacle Detection Task running logic\n");

        printf("Getting Sensor Inputs...\n");
        ReadSensors(&sensorData); // Get sensor inputs
        
        printf("Overriding Motor Commands...\n");
        DodgeAction = Obstacle_Update(&sensorData); // Determine action to take 
        xQueueOverwrite(xMotorQueue, &DodgeAction); // Override actions from TrackUser_Task
    }
}

void TrackUser_Task(void)
{
    RobotAction_t FollowAction;

    for (;;) {
        //High Prio
        printf("User Tracking Task running logic\n");
        printf("Getting UWB Inputs...\n");
        printf("Giving Motor Commands...\n");

    }
}



