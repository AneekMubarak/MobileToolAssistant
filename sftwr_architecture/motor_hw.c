#include "motor_hw.h"
#include "main.h"

/*
For the motor definitions,

After you:
1. Open CubeMX (Pinout view)
2. Click on a GPIO pin (example: PA5)
3. Set it as GPIO_Output
4. Give it a User Label (for example: RIGHT_DIR)

after which main.h will have:
#define RIGHT_DIR_Pin GPIO_PIN_5
#define RIGHT_DIR_GPIO_Port GPIOA

*/
void MotorHW_SetLeftSpeed(int speed)
{
    if (speed > 0)
    {
        // Left Motor Forward
        HAL_GPIO_WritePin(LEFT_DIR_GPIO_Port, LEFT_DIR_Pin, GPIO_PIN_SET);
    }
    else
    {
        // Left Motor Reverse
        HAL_GPIO_WritePin(LEFT_DIR_GPIO_Port, LEFT_DIR_Pin, GPIO_PIN_RESET);
        speed = -speed; // make positive for PWM
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
}

void MotorHW_SetRightSpeed(int speed)
{
    if (speed > 0)
    {
        // Right Motor Forward
        HAL_GPIO_WritePin(RIGHT_DIR_GPIO_Port, RIGHT_DIR_Pin, GPIO_PIN_SET);
    }
    else
    {
        // Right Motor Reverse
        HAL_GPIO_WritePin(RIGHT_DIR_GPIO_Port, RIGHT_DIR_Pin, GPIO_PIN_RESET);
        speed = -speed;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
}

void MotorHW_StopAll(void)
{
    // Stop both motors
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

// Logic Part
void Motor_ExecuteAction(RobotAction_t action)
{
    switch(action)
    {
        case ACTION_MOVE_FORWARD:
            MotorHW_SetLeftSpeed(BASE_SPEED);
            MotorHW_SetRightSpeed(BASE_SPEED);
            break;

        case ACTION_TURN_LEFT:
            MotorHW_SetLeftSpeed(-BASE_SPEED);
            MotorHW_SetRightSpeed(BASE_SPEED);
            break;

        case ACTION_TURN_RIGHT:
            MotorHW_SetLeftSpeed(BASE_SPEED);
            MotorHW_SetRightSpeed(-BASE_SPEED);
            break;

        case ACTION_STOP:
        default:
            MotorHW_StopAll();
            break;
    }
}
