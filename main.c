/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
unsigned long Weight_Shiwu = 0;//读取到力的大小
unsigned long Weight_KG = 0;//转换成的KG
uint8_t data[3]={0x01,0x01,0x01};//输出24位AD转换后的值
//校准参数
//因为不同的传感器特性曲线不是很一致，因此，每一个传感器需要矫正这里这个参数才能使测量值很准确。
//当发现测试出来的重量偏大时，增加该数值。
//2100对应的是1kg量程传感器的参数值
//（3kg量程在750左右，5kg量程在430左右，
//10kg量程在210左右，20kg在110左右）
//测量结果偏大），增加改值进行校准，一般每差1g，此数值增加10进行校准
//如果测试出来的重量偏小时，减小改数值。
//该值可以为小数
#define GapValue 110
#include "stdio.h"
#include <string.h>
#include "./OLED/oled.h"
#include "./OLED/bmp.h"
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
unsigned long HX711_Read(void);	//增益128,读取HX711转换值
void delay_us(uint8_t value);//延时函数
void Get_Weight(void);//得到压力传感器数值
void hexToDecimal(uint8_t *data, int length, char *str);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int sametime = 0;  //体重读数相同的次数
int samenum = 8;   //最大体重相同次数
uint8_t aRxBuffer;			//接收中断缓冲
uint8_t Uart1_RxBuff[256];		//接收缓冲
unsigned long baseweight = 0; //没有载重时候的值
uint8_t Flag_ERROR = 0;
//void HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer, 1);  //串口中断接收函数
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);//让交互力传感器工作在正常状态	
	
	char numberstr[50];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

		OLED_Init();			//初始化OLED  
		OLED_Clear(); 
		OLED_ShowString(24,1,"weight:",16);
//		OLED_ShowCHinese(0,0,0);//中
//		OLED_ShowCHinese(18,0,1);//景
//		OLED_ShowCHinese(36,0,2);//园
//		OLED_ShowCHinese(54,0,3);//电
//		OLED_ShowCHinese(72,0,4);//子
//		OLED_ShowCHinese(90,0,5);//科
//		OLED_ShowCHinese(108,0,6);//技
	HAL_UART_Receive_IT(&huart2, &aRxBuffer, 1);   
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		Get_Weight();//获取当前交互力的值
		
		
		//减去偏置值，并转换成KG
		if( Weight_Shiwu > baseweight)
		{
			Weight_KG = Weight_Shiwu - baseweight;
		}
		else
		{
			Weight_KG = 0;
		}
		
		float tr = (float)(Weight_KG)/165;
		Weight_KG = Weight_KG/16;
		


		//如果重量大于一定值再显示	
		if(Weight_KG>=1)
		{

			//显示体重
			OLED_ShowString(24,1,"weight:",16);
			

			sprintf(numberstr, "%.1f", tr);
			OLED_ShowString(35,4,numberstr,16);
			
			OLED_ShowString(80,4,"KG",16);
			
			//OLED_ShowNum(24,4,Weight_KG,4,16);
			

			//将体重数值发送到手机蓝牙
			HAL_UART_Transmit(&huart2, (uint8_t *)numberstr, strlen(numberstr), 0xFFFF);
			
			
			
			HAL_Delay(500);
			
		}
		else
		{
			//清屏
			OLED_Clear();
			
			//将体重数值发送到手机蓝牙
			tr = 0.0;
			sprintf(numberstr, "%.1f", tr);
			HAL_UART_Transmit(&huart2, (uint8_t *)numberstr, strlen(numberstr), 0xFFFF);
		}

		

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	  //HAL_UART_Transmit(&huart2, (uint8_t *)numberstr, strlen(numberstr)+8, 0xFFFF);
	  if(((uint8_t*)&aRxBuffer)[0]=='A')
		{
			baseweight = Weight_Shiwu;
			HAL_UART_Transmit(&huart2, &aRxBuffer, sizeof(aRxBuffer),0xFFFF);
   
		}
						 
		//重新开启中断接收
		HAL_UART_Receive_IT(&huart2, &aRxBuffer, 1);



}

//十六进制转10进制
void hexToDecimal(uint8_t *data, int length, char *str)
{
    for (int i = 0; i < length; i++) {
        sprintf(&str[i * 3], "%03d", data[i]);
    }
}

//****************************************************
//延时us函数
//****************************************************
void delay_us(uint8_t value)
{
	uint8_t i;
	for(i=value;i>0;i--);
}



//****************************************************
//读取HX711,对第一个交互力传感器
//****************************************************
unsigned long HX711_Read(void)	//增益128
{
		unsigned long count; 
		unsigned char i; 
	
  	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);//模块没准备好时，单片机在CLK线上输出低电平
  	count=0;
		while(1)
		{
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==0)
			{
				delay_us(100);
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==0)
				{
					break;
				}			
			}
		}
		
		HAL_Delay(10);
  	for(i=0;i<24;i++)
	{ 			
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
			delay_us(100);
	  	count=count<<1; 	
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);		
	  	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==1)//读入数据
			{
				count++; 
			}		
	} 
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
	  delay_us(20);
    count=count^0x800000;//第25个脉冲下降沿来时，转换数据
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
		return(count);
}

//****************************************************
//对第一个传感器称重
//****************************************************
void Get_Weight(void)
{
	Weight_Shiwu = HX711_Read();
						  
	data[0]=(Weight_Shiwu&0xFF0000)>>16;
	data[1]=(Weight_Shiwu-(data[0]<<16))>>8;
	data[2]=(Weight_Shiwu&0x0000FF);

	if(Weight_Shiwu > 0)			
	{	
		Weight_Shiwu = (unsigned int)((float)Weight_Shiwu/GapValue - 79000); 	//计算实物的实际重量																		
																		
		if(Weight_Shiwu > 5000)		//超重报警
		{
			Flag_ERROR = 1;	
		}
		else
		{
			Flag_ERROR = 0;
		}
	}
	else
	{
		Weight_Shiwu = 0;
		Flag_ERROR = 1;				//负重报警
	}
	
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
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
