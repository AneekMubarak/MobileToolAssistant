#include "ObstDet_hw.h"

#define OBSTACLE_THRESHOLD 1000 //1000mm == 1m

static uint16_t filtered_left = 0;
static uint16_t filtered_center = 0;
static uint16_t filtered_right = 0;

static uint16_t Filter(uint16_t previous, uint16_t current)
{
    return 0.7 * previous + 0.3 * current;
}

void Obstacle_Init(void)
{
    filtered_left = 0;
    filtered_center = 0;
    filtered_right = 0;
}

void ObstDet_HW_Init(void)
{
    // Initialize each sensor
    VL53L1X_Init(LEFT_ADDR);
    VL53L1X_Init(CENTER_ADDR);
    VL53L1X_Init(RIGHT_ADDR);

    VL53L1X_StartRanging(LEFT_ADDR);
    VL53L1X_StartRanging(CENTER_ADDR);
    VL53L1X_StartRanging(RIGHT_ADDR);
}


RobotAction_t Obstacle_Update(ToF_Data_t *data)
{
    filtered_left   = Filter(filtered_left, data->left_mm);
    filtered_center = Filter(filtered_center, data->center_mm);
    filtered_right  = Filter(filtered_right, data->right_mm);

    if (filtered_center < OBSTACLE_THRESHOLD)
    {
        if (filtered_left > filtered_right)
            return ACTION_TURN_LEFT;
        else
            return ACTION_TURN_RIGHT;
    }

    if (filtered_left < OBSTACLE_THRESHOLD)
        return ACTION_TURN_RIGHT;

    if (filtered_right < OBSTACLE_THRESHOLD)
        return ACTION_TURN_LEFT;

    return ACTION_MOVE_FORWARD;
}

// void ReadSensors(ToF_Data_t *data)
// {
//     VL53L1X_GetDistance(LEFT_ADDR,   &data->left_mm);
//     VL53L1X_GetDistance(CENTER_ADDR, &data->center_mm);
//     VL53L1X_GetDistance(RIGHT_ADDR,  &data->right_mm);
// }

// This version prevents blocking, checking if it is actually ready
void ReadSensors(ToF_Data_t *data)
{
    uint8_t ready;

    VL53L1X_CheckForDataReady(LEFT_ADDR, &ready);
    if (ready)
    {
        VL53L1X_GetDistance(LEFT_ADDR, &data->left_mm);
        VL53L1X_ClearInterrupt(LEFT_ADDR);
    }

    VL53L1X_CheckForDataReady(CENTER_ADDR, &ready);
    if (ready)
    {
        VL53L1X_GetDistance(CENTER_ADDR, &data->center_mm);
        VL53L1X_ClearInterrupt(CENTER_ADDR);
    }

    VL53L1X_CheckForDataReady(RIGHT_ADDR, &ready);
    if (ready)
    {
        VL53L1X_GetDistance(RIGHT_ADDR, &data->right_mm);
        VL53L1X_ClearInterrupt(RIGHT_ADDR);
    }
}





/*
// In ToF FreeRTOS task

ToF_Data_t sensorData;
RobotAction_t action;

for (;;)
{
    // Read real sensors here

    action = Obstacle_Update(&sensorData);

    xQueueSend(xMotorQueue, &action, 0);

    vTaskDelay(pdMS_TO_TICKS(30));
}


// When getting data:

VL53L1X_GetDistance(LEFT_ADDR, &sensorData.left_mm);
VL53L1X_GetDistance(CENTER_ADDR, &sensorData.center_mm);
VL53L1X_GetDistance(RIGHT_ADDR, &sensorData.right_mm);

*/