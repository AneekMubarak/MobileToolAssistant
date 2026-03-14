#ifndef OBST_DET_H
#define OBST_DET_H

#include "main.h"
#include "motor_hw.h"
#include "vl53l1x_api.h"

/* ================= Sensor Count ================= */

#define TOF_SENSOR_COUNT 3

enum
{
    TOF_LEFT = 0,
    TOF_CENTER = 1,
    TOF_RIGHT = 2
};

/* ================= Default Address ================= */

#define TOF_DEFAULT_ADDR 0x52

/* ================= Sensor Addresses ================= */

#define TOF_L_ADDR 0x54
#define TOF_C_ADDR 0x56
#define TOF_R_ADDR 0x58


/* ================= XSHUT Pins ================= */

#define XSHUT_L_PORT GPIOA
#define XSHUT_C_PORT GPIOA
#define XSHUT_R_PORT GPIOA

#define XSHUT_L_PIN GPIO_PIN_1
#define XSHUT_C_PIN GPIO_PIN_2
#define XSHUT_R_PIN GPIO_PIN_3

/* ================= Obstacle Dodging Threshold ================= */

#define OBSTACLE_THRESHOLD 500 //1000mm == 1m

/* ================= Sensor Data Structure ================= */

typedef struct
{
    uint16_t distance;
    uint8_t sensorState;
    uint8_t dataReady;
    uint8_t rangeStatus;

} ToF_Data_t;


/* ================= Global Sensor Array ================= */

extern ToF_Data_t ToF_DATA[TOF_SENSOR_COUNT];

/* ================= Sensor Address Array ================= */

extern const uint16_t tofAddresses[TOF_SENSOR_COUNT];


/* ================= Robot Dodge Action ================= */

extern RobotAction_t dodgeAction;

/* ================= Public Functions ================= */

void ToF_ResetData(ToF_Data_t *sensor);
void ToF_AssignAddresses(void);
int  ToF_InitSensor(uint16_t dev, ToF_Data_t *sensor);
void ToF_UpdateSensors(void);
void ToF_SetDodgeAction(void);


/* ================= Distance Access ================= */

uint16_t ToF_GetLeftDistance(void);
uint16_t ToF_GetCenterDistance(void);
uint16_t ToF_GetRightDistance(void);

#endif


