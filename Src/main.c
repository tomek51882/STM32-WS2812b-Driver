/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t zero = 0b11000000;
uint8_t one = 0b11111000;
typedef struct WS2812B
{
	uint8_t red;
	uint8_t green;
	uint8_t blue;
}WS2812B;
typedef enum { false, true } bool;


static uint8_t buffer[24];
static WS2812B strip[1];
static bool isConnected;
static bool isConnecting;
static uint8_t displayBufferLine1[18];
static uint8_t displayBufferLine2[18];
static uint8_t displayBufferLine3[18];
static uint8_t displayBufferLine4[18];
//static uint8_t dataReceived[8];
static uint8_t instReceived[3];
uint8_t selected;
uint8_t sendSelected;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

uint8_t step;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void SetColor(uint8_t,uint8_t,uint8_t,uint8_t);
void SetColorFromWS2812B(uint8_t, WS2812B);
void StripInit(void);
void StripShow(void);
void StripReset(void);
void rainbowCycle(uint8_t);
WS2812B Wheel(unsigned char);

void ConnectToArduino(void);
void CheckACK(void);
void GetDisplayData(void);
void CheckSDD(void);
void HAL_GPIO_EXTI_Callback(uint16_t);
void SSD1306_RefreshUI(void);
void DownloadFirstPattern(void);
void DownloadSecondPattern(void);
void DownloadThirdPattern(void);
void DownloadFourthPattern(void);
void testt(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  isConnecting = false;
  isConnected = false;

  step=0;
  selected=-1;
  sendSelected=0;

  SSD1306_Init ();
  SSD1306_Fill (0);  // fill the display with black color
  SSD1306_GotoXY(0,0);
  SSD1306_Puts("Test wirelessUART", &Font_7x10 , 1);
  SSD1306_DrawCircle(120,25,4,1);
  SSD1306_DrawCircle(120,35,4,1);
  SSD1306_DrawCircle(120,45,4,1);
  SSD1306_DrawCircle(120,55,4,1);
  SSD1306_UpdateScreen();


  ConnectToArduino();
  CheckACK();
  GetDisplayData();
  CheckSDD();
  testt();
  HAL_UART_Transmit(&huart3,"ACK",3,250);
  HAL_UART_Receive(&huart3, instReceived, 3, 1000);
  SSD1306_RefreshUI();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(sendSelected==1)
	  {
		  DownloadFirstPattern();
	  }
	  if(sendSelected==2)
	  {
		  DownloadSecondPattern();
	  }
	  if(sendSelected==3)
	  {
		  DownloadThirdPattern();
	  }
	  if(sendSelected==4)
	  {
		  DownloadFourthPattern();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 1200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void StripInit()
{
	size_t n = sizeof(strip)/sizeof(strip[0]);

	for(int i =0;i<n;i++)
	{
		strip[i].red=0;
		strip[i].green=0;
		strip[i].blue=0;
	}
}
void SetColor(uint8_t index, uint8_t r,uint8_t g, uint8_t b)
{
	strip[index].red=r;
	strip[index].green=g;
	strip[index].blue=b;
}
void SetColorFromWS2812B(uint8_t index, WS2812B newDiode)
{
	strip[index].red = newDiode.red;
	strip[index].green = newDiode.green;
	strip[index].blue = newDiode.blue;
}
void StripReset()
{
	uint8_t reset[50];
	for(uint8_t i = 0; i < 50; i++)
	{
		reset[i] = 0x00;
	}
	HAL_SPI_Transmit(&hspi1, reset, 50,250);
}
void StripShow()
{
	size_t n = sizeof(strip)/sizeof(strip[0]);
	for(int i=0;i<n;i++)
	{
		//GREEN
		for(int j=7;j>=0;j--)
		{
			if((strip[i].green & (1<<j)) ==0)
			{
				buffer[(7-j)+(i*24)]=zero;
			}
			else
			{
				buffer[(7-j)+(i*24)]=one;
			}
		}
		//RED
		for(int j=7;j>=0;j--)
		{
			if((strip[i].red & (1<<j)) ==0)
			{
				buffer[(7-j+8)+(i*24)]=zero;
			}
			else
			{
				buffer[(7-j+8)+(i*24)]=one;
			}
		}
		//BLUE
		for(int j=7;j>=0;j--)
		{
			if((strip[i].blue & (1<<j)) ==0)
			{
				buffer[(7-j+16)+(i*24)]=zero;
			}
			else
			{
				buffer[(7-j+16)+(i*24)]=one;
			}
		}
	}
	HAL_SPI_Transmit(&hspi1,buffer,sizeof(buffer),250);
}
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;
  size_t n = sizeof(strip)/sizeof(strip[0]);
  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< n; i++) {
    	SetColorFromWS2812B(i, Wheel(((i * 256 / n) + j) & 255));
    }
    StripShow();
    HAL_Delay(wait);
  }
}
WS2812B Wheel(unsigned char WheelPos) {
  WheelPos = 255 - WheelPos;
  WS2812B temp;
  if(WheelPos < 85)
  {
	  temp.red = 255 - WheelPos * 3;
	  temp.green = 0;
	  temp.blue = WheelPos * 3;
	  return temp;
  }
  if(WheelPos < 170)
  {
    WheelPos -= 85;
    temp.red = 0;
	temp.green = WheelPos * 3;
	temp.blue = 255 - WheelPos * 3;
	return temp;
  }
  WheelPos -= 170;
  temp.red = WheelPos * 3;
  temp.green = 255 - WheelPos * 3;
  temp.blue = 0;
  return temp;
}

void CheckACK()
{

	char _temp[3]={'A','C','K'};
	//Check if received data is from IAR call
	for(int i=0;i<3;i++)
	{
		if(instReceived[i]!=_temp[i])
		{
			if(isConnecting==true)
			{
				isConnecting=false;
				return;
			}
		}
	}
	if(isConnecting==true)
	{
		isConnected=true;
		isConnecting=false;
	}
	HAL_UART_Transmit(&huart3,"ACK",3,250);
	HAL_UART_Receive(&huart3, instReceived, 3, 1000);
}
void ConnectToArduino()
{
	if(isConnecting == false)
	{
		isConnecting = true;
		HAL_UART_Transmit(&huart3,"SYN",3,250);
		//HAL_UART_Transmit_IT(&huart3,"IAR",3);

		HAL_UART_Receive(&huart3, instReceived, 3, 1000);
		//HAL_UART_Receive_IT(&huart3,instReceived,sizeof(instReceived));
	}
}
void GetDisplayData()
{
	if(isConnected==true)
	{
		HAL_UART_Transmit(&huart3,"GDD",3,250);
		HAL_UART_Receive(&huart3, instReceived, 3, 1000);

		char _temp[3]={'R','D','D'};
		for(int i=0;i<3;i++)
		{
			if(instReceived[i]!=_temp[i])
			{
				if(isConnecting==true)
				{
					isConnecting=false;
					return;
				}
			}
		}
		HAL_UART_Transmit(&huart3,"SD1",3,250);
		HAL_UART_Receive(&huart3, displayBufferLine1, sizeof(displayBufferLine1), 1000);

		HAL_UART_Transmit(&huart3,"SD2",3,250);
		HAL_UART_Receive(&huart3, displayBufferLine2, sizeof(displayBufferLine2), 1000);

		HAL_UART_Transmit(&huart3,"SD3",3,250);
		HAL_UART_Receive(&huart3, displayBufferLine3, sizeof(displayBufferLine3), 1000);

		HAL_UART_Transmit(&huart3,"SD4",3,250);
		HAL_UART_Receive(&huart3, displayBufferLine4, sizeof(displayBufferLine4), 1000);

		SSD1306_GotoXY(0,20);
		SSD1306_Puts((char *)displayBufferLine1, &Font_7x10 , 1);
		SSD1306_GotoXY(0,30);
		SSD1306_Puts((char *)displayBufferLine2, &Font_7x10 , 1);
		SSD1306_GotoXY(0,40);
		SSD1306_Puts((char *)displayBufferLine3, &Font_7x10 , 1);
		SSD1306_GotoXY(0,50);
		SSD1306_Puts((char *)displayBufferLine4, &Font_7x10 , 1);
	}
}
void CheckSDD()
{
	if(isConnected==true)
	{
		HAL_UART_Transmit(&huart3,"OK1",3,250);
		HAL_UART_Receive(&huart3, instReceived, 3, 1000);
	}
}
void testt()
{
	if(isConnected==true)
	{
		HAL_UART_Transmit(&huart3,"ERR",3,250);
		HAL_UART_Receive(&huart3, instReceived, 3, 1000);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	if(GPIO_Pin == GPIO_PIN_13)
	{
		if(selected==0)
		{
			HAL_UART_Transmit(&huart3,"GD1",3,250);
			HAL_UART_Receive(&huart3, instReceived, 3, 1000);
			sendSelected=1;
		}
		if(selected==1)
		{
			//DownloadSecondPattern();
			sendSelected=2;
		}
		if(selected==2)
		{
			//DownloadThirdPattern();
			sendSelected=3;
		}
		if(selected==3)
		{
			//DownloadFourthPattern();
			sendSelected=4;
		}
	}
	if(GPIO_Pin == GPIO_PIN_9)
	{
		selected++;
		if(selected>3)
		{
			selected=0;
		}
		SSD1306_RefreshUI();
		if(selected==0)
		{
			SSD1306_DrawFilledCircle(120,25,4,1);
		}
		if(selected==1)
		{
			SSD1306_DrawFilledCircle(120,35,4,1);
		}
		if(selected==2)
		{
			SSD1306_DrawFilledCircle(120,45,4,1);
		}
		if(selected==3)
		{
			SSD1306_DrawFilledCircle(120,55,4,1);
		}
		SSD1306_UpdateScreen();
	}
}
void SSD1306_RefreshUI()
{
	SSD1306_DrawFilledRectangle(115,20,10,60,0);
	SSD1306_DrawCircle(120,25,4,1);
    SSD1306_DrawCircle(120,35,4,1);
    SSD1306_DrawCircle(120,45,4,1);
    SSD1306_DrawCircle(120,55,4,1);
    SSD1306_UpdateScreen();
}
void DownloadFirstPattern()
{
	if(isConnected==true)
	{
		HAL_UART_Transmit(&huart3,"GD1",3,250);
		HAL_UART_Receive(&huart3, instReceived, 3, 1000);
		sendSelected=0;
	}
}
void DownloadSecondPattern()
{
	if(isConnected==true)
	{
		HAL_UART_Transmit(&huart3,"GD1",3,250);
		sendSelected=0;
	}
}
void DownloadThirdPattern()
{
	if(isConnected==true)
	{
		HAL_UART_Transmit(&huart3,"GD1",3,250);
		sendSelected=0;
	}
}
void DownloadFourthPattern()
{
	if(isConnected==true)
	{
		HAL_UART_Transmit(&huart3,"GD1",3,250);
		sendSelected=0;
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
