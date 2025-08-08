/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void command(char *cmd);
void servoSetAngle(TIM_HandleTypeDef *htim, uint32_t channel, int angle);
void adxl345_init(I2C_HandleTypeDef *hi2c);
void adxl345_read_xyz(I2C_HandleTypeDef *hi2c, int16_t *x, int16_t *y, int16_t *z);
void calc_acceleration_ms2(int16_t xraw, int16_t yraw, int16_t zraw, float* ax, float* ay, float* az);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LIVE_EXPRESSION // KAPAT - AÇ.
#define FLASH_USER_PAGE_ADDR ((uint32_t)0x0801FC00) // son page başlangıç
// #define FLASH_PAGE_SIZE	0x400 // 1 KBs
#define RX_BUFFER_SIZE 128
#define ADC_CHANNEL_COUNT 3
#define ADXL345_I2C_ADDR (0x53<<1)

#define ADXL345_REG_POWER_CTL 0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATAX0 0x32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint8_t adcDataReady = 0;
uint16_t adcValues[ADC_CHANNEL_COUNT]; // PA2, PA3, PA4 sırasıyla
char rxBuffer[RX_BUFFER_SIZE];
char exampleCommands[] =
    "-------------- EXAMPLE COMMANDS -----------\r\n"
    "LED;ON          \r\n"
    "LED;OFF         \r\n"
    "LED;STATUS;     \r\n"
    "PWM;SET;50      \r\n"
    "SERVO;SET;30    \r\n"
	"MCU;RESTART;    \r\n"
	"I2C;SCAN;		 \r\n"
	"I2C;DATA;<gX,gY,gZ,aX,aY,aZ> \r\n"
	"I2C;DATA;<gALL,aALL> 	\r\n"
	"ADXL;SERVO;Y;<ON-OFF> \r\n"
	"ADXL;ACC;LED;<ON-OFF> \r\n"
	"ADC;SHOW;<1-3>  \r\n"
	"ADC;SHOW;ALL    \r\n"
	"ADC;SHOW;RESISTOR;\r\n"
	"ADC;MEASURE;CAP; \r\n"
	"FLASH;WRITE;<address>;<data> \r\n" // adress = 0x0801FC00
	"FLASH;READ;<address>   \r\n"
	"PWM;SET;FREQ;<frekans>;DUTY;<%duty>; \r\n"
	"WAIT;<timeout ms>   \r\n"
	"HELP;           \r\n"
    "------------------------------------------\r\n";
uint8_t rxData;
uint8_t rxIndex = 0; //rxBuffer'a kacinci karakter yazildiginin kontrolu
bool process_uart = false;
int16_t gx = 0, gy = 0, gz = 0;
float ax= 0, ay= 0, az= 0;
volatile bool servoFollowY = false;
bool ledFollowAccX = false;

//PWM SET FREQ DEGISKENLERI
volatile uint32_t ic_rising1 = 0, ic_rising2 = 0, ic_falling = 0;
volatile uint32_t period = 0, high_time = 0;
volatile float measured_freq = 0, measured_duty = 0;
volatile uint8_t first_rising = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rxData, 1);
 // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_UART_Transmit(&huart1, (uint8_t*)exampleCommands, strlen(exampleCommands), HAL_MAX_DELAY);
  adxl345_init(&hi2c1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2); // PA7 - Rising
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3); // PB0 - Falling

 // IWDG_Init();
  // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 5);
  //HAL_UART_Transmit(&huart1, (uint8_t*)"hello\n\r", 20, 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  char msg[] = "deneme123\r\n";
//	      HAL_UART_Transmit(&huart1, (uint8_t*)msg, sizeof(msg)-1, HAL_MAX_DELAY);
//	      HAL_Delay(1000);

	 HAL_IWDG_Refresh(&hiwdg); // Burada sürekli besle
	          // WAIT komutu dışında


	  if(process_uart)
	  {
		  command(rxBuffer);								// komutu işle

	  }
	#ifdef LIVE_EXPRESSION
    // DEBUG MODU DEĞERLER SÜREKLİ GÜNCELLENİYOR
    adxl345_read_xyz(&hi2c1, &gx, &gy, &gz);
    calc_acceleration_ms2(gx, gy, gz, &ax, &ay, &az);
    #endif

    	if (servoFollowY)
    	{
    		float sinirY =ay;
    		if(sinirY> 9.81f) sinirY = 9.81f;
    		if(sinirY< -9.81f) sinirY = -9.81f;

    		float angle = (sinirY / 9.81f) * 90.0f;
    		if(angle>90) angle = 90;
    		if(angle<-90) angle = -90;

    		servoSetAngle(&htim2, TIM_CHANNEL_2, (int)angle);
    		HAL_Delay(20);
    	}

    	if (ledFollowAccX)
    	{
    		if(ax > 10.0f || ax < -10.0f){
    			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    		}
    		else{
    			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : RED_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void set_pwm_freq_duty(uint32_t freq, uint8_t duty) {
    uint32_t timer_clk = 72000000;  // 72 MHz
    uint32_t prescaler = 0; // PSC=0 --> 1
    uint32_t period = (timer_clk / (prescaler + 1)) / freq - 1;
    uint32_t pulse = ((period + 1) * duty) / 100;

    __HAL_TIM_SET_PRESCALER(&htim3, prescaler); // Gerekirse ekle!
    __HAL_TIM_SET_AUTORELOAD(&htim3, period);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // PA6
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    static uint32_t last_rising = 0;
    static uint32_t last_falling = 0;
    static uint32_t period = 0;
    static uint32_t high_time = 0;
    static uint8_t state = 0; // 0: rising bekliyor, 1: falling bekliyor

    if (htim->Instance == TIM3) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) { // Rising
            uint32_t now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            if (state == 0) { // Doğru sıralama
                period = (now >= last_rising) ? (now - last_rising) : ((htim->Init.Period - last_rising) + now + 1);
                last_rising = now;
                state = 1; // falling bekle
            }
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) { // Falling
            uint32_t now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            if (state == 1) {
                high_time = (now >= last_rising) ? (now - last_rising) : ((htim->Init.Period - last_rising) + now + 1);
                last_falling = now;
                state = 0; //  tekrar rising bekle
            }
        }
        if (period > 0) {
            measured_freq = (float)72000000 / (htim->Init.Prescaler + 1) / period;
            measured_duty = ((float)high_time / (float)period) * 100.0f;
        }
    }
}




void Flash_Write_Word(uint32_t address, uint32_t data){
	HAL_FLASH_Unlock();

	if(address % 4 != 0){
	        HAL_UART_Transmit(&huart1, (uint8_t*)"Hatalı adres (4 byte hizalama)!\r\n", strlen("Hatalı adres (4-byte hizalama)!\r\n"), HAL_MAX_DELAY);
	        HAL_FLASH_Lock();
	        return;
	    }

	    // Adresin son page aralığında olup olmadığını kontrol et
	    if(address < FLASH_USER_PAGE_ADDR || address >= (FLASH_USER_PAGE_ADDR + FLASH_PAGE_SIZE)){
	        HAL_UART_Transmit(&huart1, (uint8_t*)"Adres son page araliginda degil!\r\n", strlen("Adres son page araliginda degil!\r\n"), HAL_MAX_DELAY);
	        HAL_FLASH_Lock();
	        return;
	    }

	//page silme
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError = 0;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = address & ~(FLASH_PAGE_SIZE-1); // page baslangici
	EraseInitStruct.NbPages = 1;
	if((HAL_FLASHEx_Erase(&EraseInitStruct, &PageError)) != HAL_OK){
	         HAL_UART_Transmit(&huart1, ((uint8_t*)"Silme basarisiz. \r\n"), strlen("Silme basarisiz. \r\n"), HAL_MAX_DELAY);
	         HAL_FLASH_Lock();
	         return;
	}

	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data) == HAL_OK)
	{
		HAL_UART_Transmit(&huart1, (uint8_t*)"Yazma basarili. \r\n",strlen("Yazma basarili. \r\n"), HAL_MAX_DELAY);
	}
	else
	{
		HAL_UART_Transmit(&huart1, (uint8_t*)"Yazma basarisiz. \r\n", strlen("Yazma basarisiz. \r\n"), HAL_MAX_DELAY);
	}

	HAL_FLASH_Lock();
}

uint32_t Flash_Read_Word(uint32_t address){
	return *(volatile uint32_t*) address;
}





void calc_acceleration_ms2(int16_t xraw, int16_t yraw, int16_t zraw, float* ax, float* ay, float* az){
	*ax = ((float)xraw / 256.0) * 9.81f;
	*ay = ((float)yraw / 256.0) * 9.81f;
	*az = ((float)zraw / 256.0) * 9.81f;
}

void adxl345_init(I2C_HandleTypeDef *hi2c){
	uint8_t data = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, ADXL345_I2C_ADDR, ADXL345_REG_POWER_CTL, 1, &data, 1, 100);

	data = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, ADXL345_I2C_ADDR, ADXL345_REG_DATA_FORMAT, 1, &data, 1, 100);
}

void adxl345_read_xyz(I2C_HandleTypeDef *hi2c, int16_t *x, int16_t *y, int16_t *z){
	uint8_t rawData[6];
	HAL_I2C_Mem_Read(&hi2c1, ADXL345_I2C_ADDR, ADXL345_REG_DATAX0, 1, rawData, 6, 100);

	*x = (int16_t)((rawData[1] << 8) | rawData[0]);
	*y = (int16_t)((rawData[3] << 8) | rawData[2]);
	*z = (int16_t)((rawData[5] << 8) | rawData[4]);
}

float measureCapacitance()
{
    // 1. PA5 çıkış moduna alındı LOW yapıldı
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_Delay(50);  // CAP 50ms'de boşaltıldı

    // 2. PA5 pinini ADC girişine çevirildi
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

// ADC PA5 channel ayarları yapıldı
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // 4. Zamanı başlat
    uint32_t startTick = HAL_GetTick();

    float voltage = 0;
    float targetVoltage = 3.3f * 0.63f;  // 63% = RC sabiti
    while (voltage < targetVoltage)
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        uint16_t adcVal = HAL_ADC_GetValue(&hadc1);
        voltage = (3.3f * adcVal) / 4095.0f;
    }

    uint32_t endTick = HAL_GetTick();
    float sure = (float)(endTick - startTick);  // ms cinsinden

    float resistance = 10.0f;  // 10kΩ kullanıyorsan
    float capacitance_uF = sure / resistance;
    return capacitance_uF;
}

void IWDG_Init(void)
{
    // LSI clock = 40
    // Prescaler = 64
	//Counter clock = 40,000 / 64 = 625 Hz
    // Timeout = (Reload + 1) / CounterClock = (4260 + 1) / 625 ≈ 6.81 sn

    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 4260;  // 6.81 sn
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        Error_Handler();
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_interrupt_time = 0;
    uint32_t current_time = HAL_GetTick();

    if (GPIO_Pin == GPIO_PIN_8)
    {
        if (current_time - last_interrupt_time < 200)
            return;
        last_interrupt_time = current_time;

        // Komutu manuel buffer'a yaz
        strcpy(rxBuffer, "ADC;SHOW;RESISTOR");
        process_uart = true;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	 if (huart->Instance == USART1){
		 if (rxData == '\n'){ 								//enter gelince komutu tamamlandı
			 rxBuffer[rxIndex] = '\0';						// null terminate
			 //command(rxBuffer);								// komutu işle
			 rxIndex = 0;
			 process_uart = true;

		 }
		 else{
			 if (rxIndex < RX_BUFFER_SIZE - 1){
				 rxBuffer[rxIndex++] = rxData;				//karakteri kaydete
			 }
			 else{
				 rxIndex = 0;								// taşma olursa sıfırla
			 }
		 }
		 HAL_UART_Receive_IT(&huart1, &rxData, 1);			// yeni veri için kesmeyi tekrar başlat
	 }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        adcDataReady = 1;
        char msg[] = "ADC Hazir!\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}
float adcToVoltage(uint16_t adcVal) {
    return (3.3f * adcVal) / 4095.0f;
}



void command(char *cmd){
	char *token1 = strtok(cmd, ";");
	char *token2 = strtok(NULL, ";");
	char *token3 = strtok(NULL, ";");
	char *token4 = strtok(NULL, ";");

	for(int i = 0; cmd[i]; i++) {
	    if (cmd[i] == '\r') cmd[i] = '\0';
	}
	if (token1 == NULL) return;
	char msg[100];
	sprintf(msg, "token1: '%s', token2: '%s', token3: '%s'\r\n",
	        token1 ? token1 : "NULL",
	        token2 ? token2 : "NULL",
	        token3 ? token3 : "NULL");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	if (strcmp(token1, "LED") == 0){
		if (token2 == NULL){
			char msg[] = "LED icin ON veya OFF giriniz.\r\n";
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			return;
		}

		if (strcmp(token2, "ON") == 0){
			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
			char msg[] = "LED acildi.\r\n";
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}
		else if (strcmp(token2, "OFF") == 0){
			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
			char msg[] = "LED kapatildi.\r\n";
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}
		else if(strcmp(token2, "STATUS") == 0){
			GPIO_PinState state = HAL_GPIO_ReadPin(RED_LED_GPIO_Port, RED_LED_Pin);
				if(state == GPIO_PIN_RESET){
					char msg[] = "LED acik durumda. \r\n";
					HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
				}
				else if(state == GPIO_PIN_SET){
					char msg[] = "LED kapali durumda. \r\n";
					HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
				}
		}
		else{
			char msg[] = "Gecersiz LED komutu. Ornek komutlar icin HELP; yaziniz. \r\n";
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}
	}

	else if(strcmp(token1, "WAIT") == 0 && token2 != NULL)
	{
	    uint32_t delayMs = atoi(token2);
	    char msg[64];
	    sprintf(msg, "Bekleme baslatiliyor: %lu ms\r\n", delayMs);
	    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	    HAL_Delay(delayMs);

	    sprintf(msg, "Bekleme tamamlandi\r\n");
	    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	    process_uart = false;
	    return;
	}
	else if (strcmp(token1, "PWM") == 0 && strcmp(token2, "SET") == 0)
	{
	    if(token3 && strcmp(token3, "FREQ") == 0 && token4) {
	        uint32_t freq = atoi(token4);
	        char *token5 = strtok(NULL, ";");
	        char *token6 = strtok(NULL, ";");
	        if(token5 && strcmp(token5, "DUTY") == 0 && token6) {
	            uint8_t duty = atoi(token6);
	            set_pwm_freq_duty(freq, duty);

	            HAL_Delay(50);

	            char msg[128];
	            sprintf(msg, "PWM set: %lu Hz, %d%%\r\nOlculen Frekans: %.1f Hz\r\nOlculen Duty: %.1f%%\r\n",
	                    freq, duty, measured_freq, measured_duty);
	            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	        }
	    }
	}

	else if(strcmp(token1, "FLASH") == 0){
		 if(token2 && strcmp(token2, "WRITE") == 0 && token3 && token4){
		        uint32_t addr = strtoul(token3, NULL, 0);
		        uint32_t data = strtoul(token4, NULL, 0);
		        Flash_Write_Word(addr, data);
		 }
		 else if(token2 && strcmp(token2, "READ") == 0 && token3){
			 uint32_t addr = strtoul(token3, NULL, 0);
			 uint32_t value = Flash_Read_Word(addr);
			 char msg[40];
			 sprintf(msg, "FLASH READ --> 0x%08lX: 0x%08lX\r\n",addr ,value);
			 HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		 }
		 else{
			 char msg[] = "FLASH komutu hatali\r\n";
			 HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		 }
		 memset(rxBuffer, 0, sizeof(rxBuffer));
		 process_uart = false;
		 return;
	}

	else if (strcmp(token1, "ADC") == 0 && strcmp(token2, "SHOW") == 0) {
	    adcDataReady = false;  // Flag'i başlat
	    HAL_ADC_Stop_DMA(&hadc1);
	    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, ADC_CHANNEL_COUNT);

	    uint32_t startTick = HAL_GetTick() + 10; // SONSUZ DÖNGÜYÜ ÖNLEMEK İÇİN
	    while (!adcDataReady) {
	        if (HAL_GetTick() > startTick) {  // 300ms timeout
	            char errMsg[] = "ADC read timeout\r\n";
	            HAL_UART_Transmit(&huart1, (uint8_t*)errMsg, strlen(errMsg), HAL_MAX_DELAY);
	            process_uart = false;
	            return;
	        }
	        HAL_Delay(1);
	    }

	    char msg[100];
	    if (strcmp(token3, "1") == 0) {
	        int len = sprintf(msg, "PA2 Voltage: %.2f V\r\n", adcToVoltage(adcValues[0]));
	        HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
	 //       for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
	 //      	                adcValues[i] = 0;
	 //      	           }
	        process_uart = false;  // Komut işlendi, flag kapat
	            return;                // Fonksiyondan çık, daha fazla işlem yapma
	    }
	    else if (strcmp(token3, "2") == 0) {
	        int len = sprintf(msg, "PA3 Voltage: %.2f V\r\n", adcToVoltage(adcValues[1]));
	        HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
	        process_uart = false;
	       // for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
	       	 //               adcValues[i] = 0;
	       	   //        }
	        process_uart = false;
	            return;
	    }
	    else if (strcmp(token3, "3") == 0) {
	        int len = sprintf(msg, "PA4 Voltage: %.2f V\r\n", adcToVoltage(adcValues[2]));
	        HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
	//        for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
	      // 	                adcValues[i] = 0;
//	       	           }
	        process_uart = false;
	            return;
	    }
	    else if (strcmp(token3, "ALL") == 0) {
	    	if (adcDataReady) {
	    	    adcDataReady = false; // bayrağı sıfırla

	    	    printf("ADC Conversion Complete\r\n");
	    	 int len = sprintf(msg, "PA2: %.2f V, PA3: %.2f V, PA4: %.2f V\r\n",
	    	           adcValues[0] * 3.3f / 4095.0f,
	    	           adcValues[1] * 3.3f / 4095.0f,
	    	           adcValues[2] * 3.3f / 4095.0f);
	    	 HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
	    	} else {
	    	    printf("ADC read timeout\r\n");
	    	}
	    //    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
	      //          adcValues[i] = 0;
	        //   }
	        process_uart = false;  // Komut işlendi, flag kapat
	            return;                // Fonksiyondan çık, daha fazla işlem yapma
	    }
	    else if (strcmp(token3, "RESISTOR") == 0) {
	        float vOut = adcToVoltage(adcValues[2]);  // PA4
	        float rFixed = 1000.0f;
	        float vIn = 3.3f;

	        if (vOut <= 0.01f || vOut >= 3.29f) {
	            int len = sprintf(msg, "Gecersiz Vout: %.2f V - \r\n", vOut);
	            HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
	            process_uart = false;
	            return;
	        }

	        float rUnknown = (rFixed) * (vOut / (vIn - vOut));
	        int len = sprintf(msg, "R degeri: %.2f Ohm\r\n", rUnknown);
	        HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
	     //   for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
	      //  	                adcValues[i] = 0;
	       // 	            }
	        process_uart = false;  // Komut işlendi, flag kapat
	            return;                // Fonksiyondan çık, daha fazla işlem yapma
	    }
	    else {
	        int len = sprintf(msg, "Hatali giris yaptiniz: %s\r\n", token3);
	        HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
	    }
	}
	else if (strcmp(token1, "ADC") == 0 && strcmp(token2, "MEASURE") == 0)
	{
	    if (token3 == NULL)
	    {
	        char msg[] = "ADC;MEASURE;CAP komutu eksik.\r\n";
	        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	        process_uart = false;
	        return;
	    }

	    if (strcmp(token3, "CAP") == 0)
	    {
	        float cap_uF = measureCapacitance();
	        char msg[64];
	        int len = sprintf(msg, "Olculen kapasite: %.2f uF\r\n", cap_uF);
	        HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
	        process_uart = false;
	        return;
	    }
	}


	else if(strcmp(token1, "ADXL") == 0 && strcmp(token2, "SERVO") == 0 && strcmp(token3,"Y") == 0){
		if (token4 == NULL){
			char msg[] = "ADXL;SERVO;<ON-OFF> yapiniz. \r\n";
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			memset(rxBuffer, 0, sizeof(rxBuffer));
			    process_uart = false;
			return;
		}
		if (strcmp(token4, "ON") == 0){
			servoFollowY = true;
			char msg[] = "SERVO Y takip modu aktif. \r\n";
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}
		else if(strcmp(token4, "OFF") == 0){
			servoFollowY = false;
			char msg[] = "SERVO Y takip modu kapatildi. \r\n";
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}
		else{
			char msg[] = "Yanlis komut girdiniz. Ornek komutlar icin HELP; yaziniz. \r\n";
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}
		memset(rxBuffer, 0, sizeof(rxBuffer));
		    process_uart = false;
		return;
	}

	else if(strcmp(token1, "ADXL") == 0 && strcmp(token2, "ACC") == 0 && strcmp(token3,"LED") == 0){
			if (token4 == NULL){
				char msg[] = "ADXL;ACC;LED;<ON-OFF> yapiniz. \r\n";
				HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
				memset(rxBuffer, 0, sizeof(rxBuffer));
				    process_uart = false;
				return;
			}
			if (strcmp(token4, "ON") == 0){
				ledFollowAccX = true;
				char msg[] = "LED-Ivme kontrol modu aktif. \r\n";
				HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			}
			else if(strcmp(token4, "OFF") == 0){
				ledFollowAccX = false;
				char msg[] = "LED-Ivme kontrol modu kapatildi. \r\n";
				HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			}
			else{
				char msg[] = "Yanlis komut girdiniz. Ornek komutlar icin HELP; yaziniz. \r\n";
				HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			}
			memset(rxBuffer, 0, sizeof(rxBuffer));
			    process_uart = false;
			return;
		}

	else if(strcmp(token1, "I2C") == 0){
	//	if(token2 == NULL || strcmp(token2, "SCAN") !=0 ){
	//		 char msg[] = "I2C komutu icin I2C;SCAN; yaziniz\r\n";
	//		 HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	//		 memset(rxBuffer, 0, sizeof(rxBuffer));
	//		        process_uart = false;
	//		        return;
		if(strcmp(token2, "SCAN")== 0){
			 char msg[] = "I2C taramasi basladi.\r\n";
			    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

			    for(uint8_t adress = 1; adress < 128; adress++) {
			    	if(HAL_I2C_IsDeviceReady(&hi2c1, (adress<<1), 2, 10) == HAL_OK){
			    		char msg[32];
			    		sprintf(msg, "I2C cihaz bulundu:0x%02x\r\n", adress);
			    		char amsg[] = "I2C taramasi tamamlandi.\r\n";
			    						    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			    		HAL_UART_Transmit(&huart1, (uint8_t*)amsg, strlen(amsg), HAL_MAX_DELAY);
			    	}
			    }
		}
	else if (strcmp(token1, "I2C") == 0 && (strcmp(token2, "DATA") == 0)){
		adxl345_read_xyz(&hi2c1, &gx, &gy, &gz);
		calc_acceleration_ms2(gx,gy,gz,&ax,&ay,&az);
		if(token3 == NULL){
			char msg[]="I2C;DATA;<gX,gY,gZ,aX,aY,aZ> yaziniz.\r\n";
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}
		else if(strcmp(token3, "gX") == 0){
			char msg[64];
			sprintf(msg, "gX: %d \r\n", gx);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}
		else if(strcmp(token3, "gY") == 0){
					char msg[64];
					sprintf(msg, "gY: %d \r\n", gy);
					HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
				}
		else if(strcmp(token3, "gZ") == 0){
					char msg[64];
					sprintf(msg, "gZ: %d \r\n", gz);
					HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
				}
		else if(strcmp(token3, "gALL") == 0){
					char msg[128];
					sprintf(msg, "gX: %d , gY: %d , gZ: %d \r\n", gx,gy,gz);
					HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
				}
		else if(strcmp(token3, "aX") == 0){
			char msg[64];
			sprintf(msg, "aX: %.2f m/s2 \r\n", ax);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}
		else if(strcmp(token3, "aY") == 0){
					char msg[64];
					sprintf(msg, "aY: %.2f m/s2 \r\n", ay);
					HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
				}
		else if(strcmp(token3, "aZ") == 0){
					char msg[64];
					sprintf(msg, "aZ: %.2f m/s2 \r\n", az);
					HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
				}
		else if(strcmp(token3, "aALL") == 0){
					char msg[64];
					sprintf(msg, "aX: %.2f m/s2 , aY: %.2f m/s2, aZ: %.2f m/s2 \r\n", ax,ay,az);
					HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
				}
		else if(strcmp(token3, "ALL") == 0){
					char msg[128];
					sprintf(msg, "gX: %d | gY: %d | gZ: %d \r\n"
								 "-----------------------------	\r\n"
								 "aX: %.2f m/s2 | aY: %.2f m/s2 | aZ: %.2f m/s2 \r\n", gx,gy,gz,ax,ay,az);
					HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}
		else {
			   char msg[] = "Hatali komut girdiniz. HELP; ile komutlara ulasabilirsiniz.";
			        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}
		memset(rxBuffer, 0, sizeof(rxBuffer));
		    process_uart = false;
		    return;
	}
}
	else if (strcmp(token1, "MCU") == 0) {
		if(token2 == NULL){
			char msg[] = "Reset atmak icin MCU;RESTART yaziniz.\r\n";
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}
		else if (strcmp(token2, "RESTART") == 0){
			char msg[] = "Sistem yeniden baslatiliyor.\r\n";
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			HAL_Delay(1000);
			NVIC_SystemReset();
		}
		else {
				char msg[] = "Gecersiz MCU komutu. Ornek komutlar icin HELP; yaziniz.\r\n";
				HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			}
	}
	else if (strcmp(token1, "SERVO") == 0)
	{
	    if (token2 == NULL || strcmp(token2, "SET") != 0)
	    {
	        char msg[] = "SERVO icin SERVO;SET; komutunu kullaniniz. \r\n";
	        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	        memset(rxBuffer, 0, sizeof(rxBuffer));
	        process_uart = false;
	        return;
	    }

	    if (token3 == NULL)
	    {
	        char msg[] = "Lutfen servo acisi giriniz (-90 ile 90 arasi)\r\n";
	        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	        return;
	    }

	    int angle = atoi(token3);
	    if (angle < -90 || angle > 90)
	    {
	        char msg[] = "Servo acisi -90 ile 90 arasinda olmalidir\r\n";
	        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	        return;
	    }

	    // Açıyı servoSetAngle ile ayarla
	    servoSetAngle(&htim2, TIM_CHANNEL_2, angle);

	    char msg[50];
	    int len = sprintf(msg, "Servo acisi ayarlandi: %d derece\r\n", angle);
	    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
	}
	else if(strcmp(token1, "HELP")== 0){
		HAL_UART_Transmit(&huart1, (uint8_t*)exampleCommands, strlen(exampleCommands), HAL_MAX_DELAY);
	}

		    else
		    {
		        char msg[] = "Bilinmeyen komut girdiniz.\r\n";
		        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		    }
	memset(rxBuffer,0,sizeof(rxBuffer));
	process_uart = false;

}

/* else if(strcmp(token1, "PWM") == 0)
		{
		if (token2 == NULL || strcmp(token2, "SET") != 0){
			char msg[] = "PWM icin SET komutunu kullaniniz\r\n";
			            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			            memset(rxBuffer, 0, sizeof(rxBuffer));
			            	process_uart = false;
			            return;
		}
		if (token3 == NULL)
		{
		            char msg[] = "LUtfen PWM icin 0-100 arası deger giriniz.\r\n";
		            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		            return;
		}

		int pwmValue = atoi(token3); // char dizisini int'e donusturur.
		if(pwmValue < 0 || pwmValue > 100)
		{
			char msg[] = "PWM degeri 0 ile 100 arasında olmalidir.";
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            return;
		}
		uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
		        uint32_t ccr_value = (pwmValue * arr) / 100;
		        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_value);

		        char msg[50];
		        int len = sprintf(msg, "PWM duty yuzde olarak ayarlandi: %d%%\r\n", pwmValue);
		        HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
		    }
		    */
void servoSetAngle(TIM_HandleTypeDef *htim, uint32_t channel, int angle) {
	if(angle < -90) angle = -90;
	if(angle > 90) angle = 90; // max ve min degerde kalsın

	int shift_angle = angle + 90;
	uint32_t pulse_width_us = 500 + ((uint32_t)shift_angle * 2000) / 180; // us cinsinden ayar -90 shifted ile 0 oluyor. 500 us OLUYOR o da.
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
	uint32_t ccr_value = pulse_width_us;
	if(ccr_value > arr) ccr_value = arr;
	 __HAL_TIM_SET_COMPARE(htim, channel, ccr_value);
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
