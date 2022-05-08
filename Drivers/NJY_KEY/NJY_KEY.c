#include "NJY_KEY.h"
#include "main.h"
#include "base.h"

uint8_t Key_row[1]={0xff};  							 //���水����ɨ�������״̬����

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
	if(KEY_ROW0_INPUT_Read == GPIO_PIN_SET)						//��1������
	{
		delay_ms(100);									//����
		if(KEY_ROW0_INPUT_Read == GPIO_PIN_SET)					//��1������
			{
				for(i = 1;i <= 4;i++)
				{
					if(i == 1)	     KEY_CLO0_OUT_LOW;	//�������ÿ�е�ƽ
					else if(i == 2)	 KEY_CLO1_OUT_LOW;
					else if(i == 3)	 KEY_CLO2_OUT_LOW;
					else if(i == 4)	 KEY_CLO3_OUT_LOW;

					if(KEY_ROW0_INPUT_Read == GPIO_PIN_RESET)
					{
						keynum = i;
						if(i == 1)	     KEY_CLO0_OUT_HIGH;	//�������ÿ�е�ƽ
						else if(i == 2)	 KEY_CLO1_OUT_HIGH;
						else if(i == 3)	 KEY_CLO2_OUT_HIGH;
						else if(i == 4)	 KEY_CLO3_OUT_HIGH;
						break;
					}
				}
			}
	}
	else if(KEY_ROW1_INPUT_Read == GPIO_PIN_SET)						//��2������
	{
		delay_ms(100);									//����
		if(KEY_ROW1_INPUT_Read == GPIO_PIN_SET)					//��1������
			{
				for(i = 1;i <= 4;i++)
				{
					if(i == 1)	     KEY_CLO0_OUT_LOW;	//�������ÿ�е�ƽ
					else if(i == 2)	 KEY_CLO1_OUT_LOW;
					else if(i == 3)	 KEY_CLO2_OUT_LOW;
					else if(i == 4)	 KEY_CLO3_OUT_LOW;

					if(KEY_ROW1_INPUT_Read == GPIO_PIN_RESET)
					{
						keynum = 4 + i;
						if(i == 1)	     KEY_CLO0_OUT_HIGH;	//�������ÿ�е�ƽ
						else if(i == 2)	 KEY_CLO1_OUT_HIGH;
						else if(i == 3)	 KEY_CLO2_OUT_HIGH;
						else if(i == 4)	 KEY_CLO3_OUT_HIGH;
						break;
					}
				}
			}
		}

	else if(KEY_ROW2_INPUT_Read == GPIO_PIN_SET)						//��3������
	{
		delay_ms(100);									//����
		if(KEY_ROW2_INPUT_Read == GPIO_PIN_SET)					//��1������
		{
			for(i = 1;i <= 4;i++)
			{
				if(i == 1)	     KEY_CLO0_OUT_LOW;	//�������ÿ�е�ƽ
				else if(i == 2)	 KEY_CLO1_OUT_LOW;
				else if(i == 3)	 KEY_CLO2_OUT_LOW;
				else if(i == 4)	 KEY_CLO3_OUT_LOW;

				if(KEY_ROW2_INPUT_Read == GPIO_PIN_RESET)
				{
					keynum = 8 + i;
					if(i == 1)	     KEY_CLO0_OUT_HIGH;	//�������ÿ�е�ƽ
					else if(i == 2)	 KEY_CLO1_OUT_HIGH;
					else if(i == 3)	 KEY_CLO2_OUT_HIGH;
					else if(i == 4)	 KEY_CLO3_OUT_HIGH;
					break;
				}
			}
		}
	}
	else if(KEY_ROW3_INPUT_Read == GPIO_PIN_SET)						//��4������
		{
			delay_ms(100);									//����
			if(KEY_ROW3_INPUT_Read == GPIO_PIN_SET)					//��1������
			{
				for(i = 1;i <= 4;i++)
				{
					if(i == 1)	     KEY_CLO0_OUT_LOW;	//�������ÿ�е�ƽ
					else if(i == 2)	 KEY_CLO1_OUT_LOW;
					else if(i == 3)	 KEY_CLO2_OUT_LOW;
					else if(i == 4)	 KEY_CLO3_OUT_LOW;

					if(KEY_ROW3_INPUT_Read == GPIO_PIN_RESET)
						{
							keynum = 12 + i;
							if(i == 1)	     KEY_CLO0_OUT_HIGH;	//�������ÿ�е�ƽ
							else if(i == 2)	 KEY_CLO1_OUT_HIGH;
							else if(i == 3)	 KEY_CLO2_OUT_HIGH;
							else if(i == 4)	 KEY_CLO3_OUT_HIGH;
							break;
						}
				}
			}
		}

	key_reinit();//������Ҫ��һ���������������е�ƽ�Ῠ��



		return keynum;
}
