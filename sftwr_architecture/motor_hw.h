#ifndef MOTOR_HW_H
#define MOTOR_HW_H

void MotorHW_SetLeftSpeed(int speed);   // -100 to 100
void MotorHW_SetRightSpeed(int speed);  // -100 to 100
void MotorHW_StopAll(void);

typedef enum
{
    ACTION_MOVE_FORWARD,
    ACTION_TURN_LEFT,
    ACTION_TURN_RIGHT,
    ACTION_STOP
} RobotAction_t;

#endif