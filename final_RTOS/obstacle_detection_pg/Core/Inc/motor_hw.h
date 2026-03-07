#ifndef MOTOR_HW_H
#define MOTOR_HW_H

#include "main.h"
#include "cmsis_os.h"

/* ================= PWM Limits ================= */

#define PWM_MAX 999


/* ================= Motor Control API ================= */

void MotorHW_Init(void);

void MotorHW_SetLeftSpeed(int speed);   // -1000 to 1000
void MotorHW_SetRightSpeed(int speed);  // -1000 to 1000

void MotorHW_StopAll(void);


/* ================= Robot Action ================= */
typedef enum
{
    ACTION_MOVE_FORWARD,
    ACTION_TURN_LEFT,
    ACTION_TURN_RIGHT,
    ACTION_STOP
} RobotAction_t;

/* ================= High Level Drive ================= */

void Drive_Forward(uint16_t pwm);
void Drive_Reverse(uint16_t pwm);
void Drive_TurnLeft(uint16_t pwm);
void Drive_TurnRight(uint16_t pwm);
void Drive_Stop(void);

#endif
