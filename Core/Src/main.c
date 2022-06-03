/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "base.h"
#include "lcd12864.h"
#include "arm_math.h"
#include "NJY_KEY.h"
#include "w25qxx.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct{
	arm_pid_instance_f32 S;
	float				 out;
}PidCtrlTypedef;							//pidt���ڽṹ��

PidCtrlTypedef pidCtrl;						//��ѹpid
PidCtrlTypedef pid2;						//����pid


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BUF_LEN					4000		//�������鳤��
#define PWM_PERIOD_CCR1			2000		//PWM����-����ֵ
//#define DES_VOL					1.0			//Ŀ���ѹ
#define ERR_LIMIT					0.001		//�������
#define MAX_PWM					0.78		//���ռ�ձ�
#define MIN_PWM					0.01		//���ռ�ձ�
#define Vol_step					0.1/6		//Vol����������0.1��Ӳ���ɼ�/6��
#define VtoI						1.025/2			//��ѹת��������


#define blockade HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define blockadeOFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define LED_ON HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET)
#define LED_OFF HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//�����ض���
#ifdef  __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch,FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,HAL_MAX_DELAY);
    return ch;
}
int _write(int file,char *ptr,int len)
{
    int Dataldx;

    for(Dataldx=0;Dataldx<len;Dataldx++)
    {
    	__io_putchar(*ptr++);

    }
    return len;
}
//�����ض���

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float DES_VOL = 2.0	;						//Ŀ���ѹ
volatile u8 dma_cpl_flag = 0;				//dma������ɱ�־
u16 adc_raw[BUF_LEN] = {0};					//adcԭʼ����ֵ
u16 adc_raw_copy[BUF_LEN] = {0};			//adcԭʼ����ֵ����
float cur_vol = 0;							//��ǰ��ѹ
u8 KEY=0;


float DES_INT = 2.0;						//Ŀ�����
u16 adc2_raw[BUF_LEN] = {0};				//adc2ԭʼ����ֵ
u16 adc2_raw_copy[BUF_LEN] = {0};			//adc2ԭʼ����ֵ����
float cur_int = 2.2;							//��ǰ��ѹ
int V_count = 12;								//����

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);								/*ADC-DMAȫ������ɻص�����*/
float calVol();																		/*����ԭʼ����ֵ����PWM��Ч��ѹֵ*/
void pidInit();																		/*PID��ʼ��*/
static void pidExecu(float vol);													/*PIDִ��*/

void dutyCycle();
void setDuty();																		/*��������ռ�ձ�*/
void setPidState();																	/*����pid�Զ�����*/
void keySwitch();																	/*������̸�����������*/
void keyGet();																		/*���������������ʾ*/
void numError();																	/*��ֵ������*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*ADC-DMAȫ������ɻص�����*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	UNUSED(hadc);

	memcpy((void *)adc_raw_copy,(void *)adc_raw,sizeof(adc_raw));
	memcpy((void *)adc2_raw_copy,(void *)adc2_raw,sizeof(adc2_raw));
	dma_cpl_flag = 1;
}
float V=0;
/*����ԭʼ����ֵ����PWM��Ч��ѹֵ*/
float calVol()
{
	char buf[128];
	int i;
	float res;
	float sum = 0;
	float tempVal;
	for(i=0;i<BUF_LEN;i++)
	{

		tempVal=adc_raw_copy[i] * 3.3/ 4095;
//		if(tempVal<=1.5)					{tempVal+=0.0025;}
//		else if(tempVal>1.5)				{tempVal-=0.0045;}
//		else if(tempVal>1.5&&tempVal<=2.0)	{tempVal=tempVal-0.1*tempVal+0.2;}
//		V=adc_raw_copy[i];
		sum+=tempVal;
	}

	res = sum / BUF_LEN;
//	if(res > 1 && res <= 2.5)						{res-=0.01;}
//	else if(res > 2.5 && res <= 3.1)				{res-=0.026667;}
	if(res>1)										{res-=(DES_VOL-cur_int*3.15);}
	V=res;						//ADУ׼
	cur_vol = res;
//	sprintf(buf,"%.3f",cur_vol*6);
	sprintf(buf,"%.3f",V);
	display_GB2312_string(3,1,buf);

	return res;
}

/*����ԭʼ����ֵ����PWM��Ч����ֵ*/
float calInt()
{
	char buf[128];
	int i;
	float res;
	float sum = 0;
	float tempIns;
	for(i=0;i<BUF_LEN;i++)
	{
		tempIns=adc2_raw_copy[i] * 3.3 / 4095;
//		if(tempIns<=1.5)					{tempIns+=0.042;}
//		else if(tempIns>1.5&&tempIns<=2.0)	{tempIns=tempIns-0.1*tempIns+0.2;}
		sum+=tempIns;
	}

	res = sum / BUF_LEN;
	res*=VtoI;
	cur_int = res;
	sprintf(buf,"%.5f",cur_int);
	display_GB2312_string(5,1,buf);

	return res;
}

/*PID��ʼ��*/
void pidInit()
{
	pidCtrl.S.Kp=0.05;
	pidCtrl.S.Ki=0.05;
	pidCtrl.S.Kd=0.05;
	arm_pid_init_f32(&pidCtrl.S, 1);

	pidCtrl.out=0;


}

/*PIDִ��*/
static void pidExecu(float vol)
{
	float pidErr;
	char buf[128];
	pidErr = DES_VOL - vol;
	//��������Χ��
	if(fabs(pidErr) > ERR_LIMIT)
	{
		pidCtrl.out = arm_pid_f32(&pidCtrl.S, pidErr);
		//     vol			   pidCtrl.out
		//      	  	 =
		//	��ǰռ�ձ�			���ں�ռ�ձ�
		if(pidCtrl.out * (htim3.Instance->CCR1 + 1)/vol >= (htim3.Instance->ARR+1)*MAX_PWM)
		{
			//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,PWM_PERIOD_CCR1*MAX_PWM);
			htim3.Instance->CCR1 = (PWM_PERIOD_CCR1*MAX_PWM);
		}
		else if(pidCtrl.out * (htim3.Instance->CCR1 + 1)/vol <= (htim3.Instance->ARR+1)*MIN_PWM)
		{
			//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,PWM_PERIOD_CCR1*MIN_PWM);
			htim3.Instance->CCR1 = (PWM_PERIOD_CCR1*MIN_PWM);
		}
		else
		{
			htim3.Instance->CCR1 = (u32)(pidCtrl.out * (htim3.Instance->CCR1 + 1)/vol);
		}
		sprintf(buf,"%5.2f",(pidCtrl.out * (htim3.Instance->CCR1 + 1)/vol));
		display_GB2312_string(7,1,buf);
	}
}

/*�Զ�����������*/
void autoBlock(float vol)
{
	if(vol > DES_INT)
	{
		blockade;LED_ON;
		HAL_Delay(100);

	}
	else
	{
		blockadeOFF;
		LED_OFF;
	}

}

/*��������ռ�ձ�*/
int compare = 1000;
void dutyCycle()
{
	char buf[128];

	sprintf(buf,"%04d",compare);
	display_GB2312_string(1,7,buf);
}

void setDuty()
{
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,compare);
}

/*����pid�Զ�����*/
int autoPidFlag = 0;
void setPidState()
{
	if(autoPidFlag == 0){autoPidFlag = 1;}
	else {autoPidFlag = 0;}
}
u8 i=0;
/*������̸�����������*/
void keySwitch()
{
	switch(KEY)
		 {
		 case 1:
		 {
			 blockade;
			 break;
		 }
		 case 2:
		 {
			 setPidState();
			 break;
		 }
		 case 3:
		 {
			 HAL_Delay(100);DES_VOL=Vol_step*(++V_count);break;
		 }
		 case 4:
		 {
			 HAL_Delay(100);DES_VOL=Vol_step*(--V_count);break;
		 }
		 case 5:
		 {
			 compare+=50;break;
		 }
		 case 6:
		 {
			 compare-=50;break;
		 }
		 case 7:
		 {
			 HAL_Delay(100);DES_VOL=Vol_step*(V_count+=10);break;
		 }
		 case 8:
		 {
			 HAL_Delay(100);DES_VOL=Vol_step*(V_count-=10);break;
		 }
		 case 9:
		 {
			 compare+=100;break;
		 }
		 case 10:
		 {
			 compare-=100;break;
		 }
		 case 11:
		 {
			 compare=1000;break;
		 }
		 case 12:
		 {
			 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,compare);break;
		 }
		 case 13:
		 {
			 break;
		 }
		 case 14:
		 {
			 blockadeOFF;
			 break;
		 }
		 case 15:
		 {
			 DES_INT+=0.1;
			 break;
		 }
		 case 16:
		 {
			 DES_INT-=0.1;
			 break;
		 }
		 default:break;
		 }
}

/*���������������ʾ*/
void keyGet()
{
	char buf[20];
	 KEY=key_scan();
	 keySwitch();
	 numError();

	 sprintf(buf,"%02.3f",DES_VOL*6);
	 display_GB2312_string(3,60,buf);

	 sprintf(buf,"%.2f",DES_INT);
	 display_GB2312_string(5,80,buf);


}

/*��ֵ������*/
void numError()
{
	if(DES_VOL <= 0)
	{
		DES_VOL=0;
	}
	if(DES_VOL >= 3)
	{
		DES_VOL=3                ;
	}
	if(DES_INT <= 0)
	{
		DES_INT=0;
	}
	if(DES_INT >= 2.4)
	{
		DES_INT=2.4;
	}
	if(V_count <= 0)
	{
		V_count = 0;
	}
	if(V_count >= 180)
	{
		V_count = 180;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  delay_init(84);
  JLX12864G_086_GPIOInit();					//��Ļ���ų�ʼ��
  initial_lcd();							//lcd��ʼ��
  clear_screen();							//����
  pidInit();								//pid��ʼ��
  HAL_TIM_Base_Start(&htim2);				//����tim2ʱ��
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	//����PWM�����
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);	//����PWM�����

  HAL_ADC_Start_DMA(&hadc1, (u32*)adc_raw, BUF_LEN);	//����ADC-DMA����
  HAL_ADC_Start_DMA(&hadc2, (u32*)adc2_raw, BUF_LEN);	//����ADC-DMA����
  W25QXX_Init();							//W25QXX��ʼ��
  delay_ms(50);
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,compare);
  //����ڲ�flashȫ������

  LED_OFF;

  delay_ms(500);
  setPidState();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  display_GB2312_string(1,1," ũ��ӱ ");
  while (1)
  {
	  keyGet();
	  dutyCycle();
	  if(dma_cpl_flag == 1 && autoPidFlag == 1)
	  {
		  dma_cpl_flag = 0 ;
		  pidExecu(calVol());
		  autoBlock(calInt());
		  calInt();
	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
