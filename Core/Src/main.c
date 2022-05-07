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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "base.h"
#include "lcd12864.h"
#include "arm_math.h"
#include "NJY_KEY.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct{
	arm_pid_instance_f32 S;
	float				 out;
}PidCtrlTypedef;							//pidt调节结构体

PidCtrlTypedef pidCtrl;						//pidt调节实例

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BUF_LEN					400			//采样数组长度
#define PWM_PERIOD_CCR1			8000		//PWM周期-计数值
//#define DES_VOL					1.0			//目标电压
#define ERR_LIMIT				0.05		//误差限制

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float DES_VOL = 2.0	;						//目标电压
volatile u8 dma_cpl_flag = 0;				//dma传输完成标志
u16 adc_raw[BUF_LEN] = {0};					//adc原始采样值
u16 adc_raw_copy[BUF_LEN] = {0};			//adc原始采样值备份
float cur_vol = 0;							//当前电压
u8 KEY=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*ADC-DMA全传输完成回调函数*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	UNUSED(hadc);

	memcpy((void *)adc_raw_copy,(void *)adc_raw,sizeof(adc_raw));

	dma_cpl_flag = 1;
}

/*根据原始采样值计算PWM等效电压值*/
float calVol()
{
	char buf[128];
	int i;
	float res;
	float sum = 0;
	float tempVal;
	for(i=0;i<BUF_LEN;i++)
	{
		tempVal=adc_raw_copy[i] * 3.3 / 4095;
		sum+=tempVal;
	}

	res = sum / BUF_LEN;

	cur_vol = res;
	sprintf(buf,"%.3f",cur_vol);
	display_GB2312_string(3,1,buf);

	return res;
}

/*PID初始化*/
void pidInit()
{
	pidCtrl.S.Kp=0.1;
	pidCtrl.S.Ki=0.1;
	pidCtrl.S.Kd=0.1;
	arm_pid_init_f32(&pidCtrl.S, 1);

	pidCtrl.out=0;
}

/*PID执行*/
static void pidExecu(float vol)
{
	float pidErr;

	pidErr = DES_VOL - vol;
	//误差不在允许范围内
	if(fabs(pidErr) > ERR_LIMIT)
	{
		pidCtrl.out = arm_pid_f32(&pidCtrl.S, pidErr);
		//     vol			   pidCtrl.out
		//      	  	 =
		//	当前占空比			调节后占空比
		htim3.Instance->CCR1 = (u32)(pidCtrl.out * (htim3.Instance->CCR1 + 1)/vol);
	}
}
/*矩阵键盘输入与显示*/
void keyGet()
{
	char buf[20];
	 KEY=key_scan();
	 sprintf(buf,"%d",KEY);
	 display_GB2312_string(5,1,buf);
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
  /* USER CODE BEGIN 2 */
  delay_init(84);
  JLX12864G_086_GPIOInit();					//屏幕引脚初始化
  initial_lcd();							//lcd初始化
  clear_screen();							//清屏
  pidInit();								//pid初始化
  HAL_TIM_Base_Start(&htim2);				//开启tim2时钟
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	//开启PWM波输出
  HAL_ADC_Start_DMA(&hadc1, (u32*)adc_raw, BUF_LEN);	//开启ADC-DMA传输


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  display_GB2312_string(1,1,"农杰颖");
  while (1)
  {
	  keyGet();
	  if(dma_cpl_flag == 1)
	  {
		  dma_cpl_flag = 0 ;
		  pidExecu(calVol());
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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
