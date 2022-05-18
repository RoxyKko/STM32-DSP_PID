################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/W25QXX/w25qxx.c 

OBJS += \
./Drivers/W25QXX/w25qxx.o 

C_DEPS += \
./Drivers/W25QXX/w25qxx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/W25QXX/%.o Drivers/W25QXX/%.su: ../Drivers/W25QXX/%.c Drivers/W25QXX/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx '-D__FPU_PRESENT = 1U' -D_TARGET_FPU_VFP -DARM_MATH_CM4 -c -I"D:/cubeIDE/oled_v1.6/Drivers/DSP" -I"D:/cubeIDE/oled_v1.6/Drivers/W25QXX" -I"D:/cubeIDE/oled_v1.6/Drivers/base" -I"D:/cubeIDE/oled_v1.6/Drivers/NJY_KEY" -I"D:/cubeIDE/oled_v1.6/Drivers/LCD12864" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-W25QXX

clean-Drivers-2f-W25QXX:
	-$(RM) ./Drivers/W25QXX/w25qxx.d ./Drivers/W25QXX/w25qxx.o ./Drivers/W25QXX/w25qxx.su

.PHONY: clean-Drivers-2f-W25QXX

