#ifndef __NJY_KEY_H__
#define __NJY_KEY_H__

#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>

uint8_t key_scan(void);


//横行row，纵列col

/*********列IO操作宏定义*********/
#define KEY_CLO0_OUT_LOW HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET)
#define KEY_CLO1_OUT_LOW HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET)
#define KEY_CLO2_OUT_LOW HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_RESET)
#define KEY_CLO3_OUT_LOW HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET)

#define KEY_CLO0_OUT_HIGH HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET)
#define KEY_CLO1_OUT_HIGH HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET)
#define KEY_CLO2_OUT_HIGH HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET)
#define KEY_CLO3_OUT_HIGH HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET)

/*********行IO操作宏定义*********/
#define KEY_ROW0_INPUT_Read  HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8)
#define KEY_ROW1_INPUT_Read  HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6)
#define KEY_ROW2_INPUT_Read  HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4)
#define KEY_ROW3_INPUT_Read  HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2)

#endif
