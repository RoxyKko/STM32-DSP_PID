#ifndef __LCD12864_H
#define __LCD12864_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"

#define uchar unsigned char
#define uint unsigned int
#define ulong unsigned long


//CS PE0
#define lcd_cs1_h HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET)
#define lcd_cs1_l HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET)

//RESET PB8
#define lcd_reset_h HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
#define lcd_reset_l HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)

//RS PB6
#define lcd_rs_h HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define lcd_rs_l HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)

//SDA PB4
#define lcd_sid_h HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)
#define lcd_sid_l HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)

//SCLK PG15
#define lcd_sclk_h HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_SET)
#define lcd_sclk_l HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET)

//ROM_CS PG11
#define Rom_CS_h HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_SET)
#define Rom_CS_l HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_RESET)

//ROM_SCK PG9
#define Rom_SCK_h HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET)
#define Rom_SCK_l HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET)

//ROM_IN PD4
#define Rom_IN_h HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET)
#define Rom_IN_l HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET)

//ROM_OUT PD6
#define Rom_OUT_h HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET)
#define Rom_OUT_l HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET)

//Read ROM_OUT PIN
#define	Rom_OUT_cin HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6)







void JLX12864G_086_GPIOInit(void);
void transfer_command_lcd(int data1);
void transfer_data_lcd(int data1);
void initial_lcd(void);
void lcd_address(uint page,uint column);
void clear_screen(void);
void display_128x64(uchar *dp);
void display_graphic_16x16(uchar page,uchar column,uchar *dp);
void display_graphic_8x16(uchar page,uchar column,uchar *dp);
void display_graphic_5x8(uchar page,uchar column,uchar *dp);
void send_command_to_ROM( uchar datu );
static uchar get_data_from_ROM(void);
void get_and_write_16x16(ulong fontaddr,uchar page,uchar column);
void get_and_write_8x16(ulong fontaddr,uchar page,uchar column);
void get_and_write_5x8(ulong fontaddr,uchar page,uchar column);
void display_GB2312_string(uchar page,uchar column,uchar *text);
void display_string_5x8(uchar page,uchar column,uchar *text);







#endif
