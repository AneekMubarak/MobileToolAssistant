#ifndef OBSTACLE_ALGORITHM_H
#define OBSTACLE_ALGORITHM_H

#include <stdint.h>
#include "motor_hw.h"

typedef struct
{
    uint16_t left_mm;
    uint16_t center_mm;
    uint16_t right_mm;
} ToF_Data_t;

// Sensor I2C addresses
#define LEFT_ADDR     0x52
#define CENTER_ADDR   0x54
#define RIGHT_ADDR    0x56


// Functions
void Obstacle_Init(void);
RobotAction_t Obstacle_Update(ToF_Data_t *data);
void ReadSensors(ToF_Data_t *data);

#endif
