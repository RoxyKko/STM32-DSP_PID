#include "NJY_KEY.h"
#include "main.h"
#include "base.h"

uint8_t Key_row[1]={0xff};  							 //保存按键行扫描情况的状态数组

uint8_t num=0;

void key_reinit()
{
	KEY_CLO0_OUT_HIGH;
	KEY_CLO1_OUT_HIGH;
	KEY_CLO2_OUT_HIGH;
	KEY_CLO3_OUT_HIGH;
}

uint8_t key_scan()
{
	uint8_t i,keynum=0;
	if(KEY_ROW0_INPUT_Read == GPIO_PIN_SET)						//行1被按下
	{
		delay_ms(100);									//消抖
		if(KEY_ROW0_INPUT_Read == GPIO_PIN_SET)					//行1被按下
			{
				for(i = 1;i <= 4;i++)
				{
					if(i == 1)	     KEY_CLO0_OUT_LOW;	//逐个拉低每列电平
					else if(i == 2)	 KEY_CLO1_OUT_LOW;
					else if(i == 3)	 KEY_CLO2_OUT_LOW;
					else if(i == 4)	 KEY_CLO3_OUT_LOW;

					if(KEY_ROW0_INPUT_Read == GPIO_PIN_RESET)
					{
						keynum = i;
						if(i == 1)	     KEY_CLO0_OUT_HIGH;	//逐个拉高每列电平
						else if(i == 2)	 KEY_CLO1_OUT_HIGH;
						else if(i == 3)	 KEY_CLO2_OUT_HIGH;
						else if(i == 4)	 KEY_CLO3_OUT_HIGH;
						break;
					}
				}
			}
	}
	else if(KEY_ROW1_INPUT_Read == GPIO_PIN_SET)						//行2被按下
	{
		delay_ms(100);									//消抖
		if(KEY_ROW1_INPUT_Read == GPIO_PIN_SET)					//行1被按下
			{
				for(i = 1;i <= 4;i++)
				{
					if(i == 1)	     KEY_CLO0_OUT_LOW;	//逐个拉低每列电平
					else if(i == 2)	 KEY_CLO1_OUT_LOW;
					else if(i == 3)	 KEY_CLO2_OUT_LOW;
					else if(i == 4)	 KEY_CLO3_OUT_LOW;

					if(KEY_ROW1_INPUT_Read == GPIO_PIN_RESET)
					{
						keynum = 4 + i;
						if(i == 1)	     KEY_CLO0_OUT_HIGH;	//逐个拉高每列电平
						else if(i == 2)	 KEY_CLO1_OUT_HIGH;
						else if(i == 3)	 KEY_CLO2_OUT_HIGH;
						else if(i == 4)	 KEY_CLO3_OUT_HIGH;
						break;
					}
				}
			}
		}

	else if(KEY_ROW2_INPUT_Read == GPIO_PIN_SET)						//行3被按下
	{
		delay_ms(100);									//消抖
		if(KEY_ROW2_INPUT_Read == GPIO_PIN_SET)					//行1被按下
		{
			for(i = 1;i <= 4;i++)
			{
				if(i == 1)	     KEY_CLO0_OUT_LOW;	//逐个拉低每列电平
				else if(i == 2)	 KEY_CLO1_OUT_LOW;
				else if(i == 3)	 KEY_CLO2_OUT_LOW;
				else if(i == 4)	 KEY_CLO3_OUT_LOW;

				if(KEY_ROW2_INPUT_Read == GPIO_PIN_RESET)
				{
					keynum = 8 + i;
					if(i == 1)	     KEY_CLO0_OUT_HIGH;	//逐个拉高每列电平
					else if(i == 2)	 KEY_CLO1_OUT_HIGH;
					else if(i == 3)	 KEY_CLO2_OUT_HIGH;
					else if(i == 4)	 KEY_CLO3_OUT_HIGH;
					break;
				}
			}
		}
	}
	else if(KEY_ROW3_INPUT_Read == GPIO_PIN_SET)						//行4被按下
		{
			delay_ms(100);									//消抖
			if(KEY_ROW3_INPUT_Read == GPIO_PIN_SET)					//行1被按下
			{
				for(i = 1;i <= 4;i++)
				{
					if(i == 1)	     KEY_CLO0_OUT_LOW;	//逐个拉低每列电平
					else if(i == 2)	 KEY_CLO1_OUT_LOW;
					else if(i == 3)	 KEY_CLO2_OUT_LOW;
					else if(i == 4)	 KEY_CLO3_OUT_LOW;

					if(KEY_ROW3_INPUT_Read == GPIO_PIN_RESET)
						{
							keynum = 12 + i;
							if(i == 1)	     KEY_CLO0_OUT_HIGH;	//逐个拉高每列电平
							else if(i == 2)	 KEY_CLO1_OUT_HIGH;
							else if(i == 3)	 KEY_CLO2_OUT_HIGH;
							else if(i == 4)	 KEY_CLO3_OUT_HIGH;
							break;
						}
				}
			}
		}

	key_reinit();//至关重要的一步，不重新拉高列电平会卡死



		return keynum;
}
