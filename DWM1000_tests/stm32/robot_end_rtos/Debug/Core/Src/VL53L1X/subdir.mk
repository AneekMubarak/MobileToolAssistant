################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/VL53L1X/VL53L1X_api.c \
../Core/Src/VL53L1X/VL53L1X_calibration.c \
../Core/Src/VL53L1X/vl53l1_platform.c 

OBJS += \
./Core/Src/VL53L1X/VL53L1X_api.o \
./Core/Src/VL53L1X/VL53L1X_calibration.o \
./Core/Src/VL53L1X/vl53l1_platform.o 

C_DEPS += \
./Core/Src/VL53L1X/VL53L1X_api.d \
./Core/Src/VL53L1X/VL53L1X_calibration.d \
./Core/Src/VL53L1X/vl53l1_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/VL53L1X/%.o Core/Src/VL53L1X/%.su Core/Src/VL53L1X/%.cyclo: ../Core/Src/VL53L1X/%.c Core/Src/VL53L1X/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../USB_HOST/App -I"C:/Users/ethan/Desktop/Programming/STM_stuff/MobileToolAssistant/DWM1000_tests/stm32/robot_end_rtos/Core/Inc/VL53L1X" -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-VL53L1X

clean-Core-2f-Src-2f-VL53L1X:
	-$(RM) ./Core/Src/VL53L1X/VL53L1X_api.cyclo ./Core/Src/VL53L1X/VL53L1X_api.d ./Core/Src/VL53L1X/VL53L1X_api.o ./Core/Src/VL53L1X/VL53L1X_api.su ./Core/Src/VL53L1X/VL53L1X_calibration.cyclo ./Core/Src/VL53L1X/VL53L1X_calibration.d ./Core/Src/VL53L1X/VL53L1X_calibration.o ./Core/Src/VL53L1X/VL53L1X_calibration.su ./Core/Src/VL53L1X/vl53l1_platform.cyclo ./Core/Src/VL53L1X/vl53l1_platform.d ./Core/Src/VL53L1X/vl53l1_platform.o ./Core/Src/VL53L1X/vl53l1_platform.su

.PHONY: clean-Core-2f-Src-2f-VL53L1X

