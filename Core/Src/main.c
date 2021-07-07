/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "LiquidCrystal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef unsigned char byte;
byte marioChar[] = {
        0x0E,        0x0F,
        0x0A,        0x0E,        0x04,
        0x0C,        0x04,        0x0A
};
byte flagChar[] = {
        0x08,        0x0C,
        0x0A,        0x09,        0x0A,
        0x0C,        0x08,        0x18
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Map size, w/o the end padding

uint16_t leds[8] = {GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11,
                    GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15 };

uint16_t lcd[7] = {GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11,
                   GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14};
//keypad mapping
char keys[4][4] = {
        {'D', ' ', 'R', 'P'},
        {'D', ' ', 'R', 'P'},
        {'D', ' ', 'R', 'P'},
        {'D', ' ', 'R', 'P'}
};

//keypad row & columns
uint16_t krows[4] = {GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13};
uint16_t kcols[4] = {GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4};

uint32_t outc[4] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};
uint32_t outa[4] = {GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4};


//                            >    █     円   ' '    ﾛ    x̄    flag   mario
uint8_t cellTypeChar[8] = {0x3E, 0xFF, 0xFC, 0x20, 0xDB, 0xF8, 0x01, 0x00};
uint8_t charMap[4][80];

enum cellType {BLADE, GROUND, PIPE, AIR, BLOCK, CHEST, FLAG, MARIO};
enum moveType {HALT, RUN, JUMP, RUMP} marioMode;
enum moveDir {NONE, RIGHT, UP, DOWN, UPRIGHT, DOWNRIGHT} marioDir;

int map[4][80] = {3};
// copy of the original map,
// used to revert cells when mario moves
int genmap[4][80] = {3};
int mario[2] = {2, 4};
int mapOffset = 0;

int life = 3;
// the map doesn't move for 3 periods after hit
int immunity = 0;
int coins = 0;

double volSpeed;

unsigned char uartin[1] = "";
int buttonCount = 0;
// debounce handlers
uint32_t lastUartTick = 0;
uint32_t lastButtonTick = 0;
uint32_t lastTimerTick = 0;
int isPlaying = 0;
int jumpProg = -1;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC4_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void splash(void);
void initGame(void);
void lifeLost(void);
void gameLost(void);
void gameWon(void);

void collisionCheck(enum moveDir);
enum moveDir getMoveDir(enum moveType move);
void runMario(void);
void jump(void);
void checkKeypad();

void generateMap(void);
void printMap(void);

void numToBcd(int n, int digit){
  int x[4];
//  HAL_GPIO_WritePin (GPIOA, outa[digit], 1);
  for (int i=0; i<4; i++){
    x[i] = n & (int) (pow(2, i));
    if (x[i] > 0) x[i] = 1;
    HAL_GPIO_WritePin(GPIOC, outc[i], x[i]);
  }
//  HAL_GPIO_WritePin (GPIOA, outa[digit], 0);
}

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
  MX_USB_PCD_Init();
  MX_USART1_UART_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC4_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

    //// Get seed from an unrouted ADC pin
    //// (behold the voltage randomness)
    //HAL_ADC_Start(&hadc3);
    //HAL_UART_Receive_IT(&huart1, uartin, sizeof(uartin));
    //srand(HAL_ADC_GetValue(&hadc3));
    //HAL_ADC_Stop_IT(&hadc3);
    //
    ////char data[30];
    ////sprintf(data, "%d seed \n", HAL_ADC_GetValue(&hadc3));
    ////HAL_UART_Transmit(&huart1, data, sizeof(data), 1000);

    lastButtonTick = HAL_GetTick();
    lastUartTick = HAL_GetTick();

    // LCD Init
    LiquidCrystal (GPIOB, lcd[0], lcd[1], lcd[2],
                   lcd[3], lcd[4], lcd[5], lcd[6]);
    begin(20,4);
    createChar(0, marioChar);
    createChar(1, flagChar);
    display();
    // show * Splash Screen *
    splash();
    HAL_UART_Receive_IT(&huart1, uartin, sizeof(uartin));
    HAL_GPIO_WritePin(GPIOD, krows[0] | krows [1] | krows[2] | krows[3] , 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */
  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.DMAContinuousRequests = DISABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT4_Pin */
  GPIO_InitStruct.Pin = MEMS_INT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PF4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD11 PD12 PD13
                           PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD1 PD2 PD3 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

// random map generator
// (called with initGame, srand given by time called)
void generateMap() {
    // probabilities for various cell features (/100)
    //FIXME: probability revert to add pipes/pits/blocks
    //int prob[4] = {10, 20, 10, 8};
    int prob[4] = {10, 0, 20, 0};
    for (int i=0; i<4; i++){
        switch (i) {
            // chests or blocks or sky
            case 0:
                for (int j=0; j<4; j++){
                    map[i][j] = AIR;
                    genmap[i][j] = AIR;
                    charMap[i][j] = cellTypeChar[AIR];
                }
                for (int j=4; j<79; j++){
                    int rs = rand() % 100;
                    int r;
                    if (rs <= prob[0])
                        r = CHEST;
                    else if (rs <= prob[1])
                        r = BLOCK;
                    else r = AIR;
                    map[i][j] = r;
                    genmap[i][j] = r;
                    charMap[i][j] = cellTypeChar[r];
                }
                break;
            // pipes or not
            case 2:
                for (int j=0; j<79; j++){
                    int r;
                    // if there's a block above, don't put pipes
                    if (j <= 6 || map[0][j] == BLOCK
                         || (rand()%100) > prob[2])
                        r = AIR;
                    else
                        r = PIPE;
                    map[i][j] = r;
                    genmap[i][j] = r;
                    charMap[i][j] = cellTypeChar[r];
                }
                break;
            // ground or pit
            case 3:
                for (int j=0; j<79; j++){
                    int r;
                    // if there's a pipe, don't put a pit
                    // also if there's a block
                    // check previous/next cell for pit/pipe/block too
                    if (j <= 8 || j >= 58
                    || map[2][j] == PIPE || map[2][j-1] == PIPE || map[2][j+1] == PIPE
                    || map[0][j] == BLOCK || map[0][j-1] == BLOCK || map[0][j+1] == BLOCK
                    || (rand()%100) > prob[3]
                    || map[3][j-1] == AIR)
                        r = GROUND;
                    else
                        r = AIR;
                    map[i][j] = r;
                    genmap[i][j] = r;
                    charMap[i][j] = cellTypeChar[r];
                }
                break;
            case 1:
            default:
                for(int j=0; j<79; j++){
                    map[i][j] = AIR;
                    genmap[i][j] = AIR;
                    charMap[i][j] = cellTypeChar[AIR];
                }
                break;
        }
    }
    map[2][59] = FLAG;
    genmap[2][59] = FLAG;
    charMap[2][59] = cellTypeChar[FLAG];

}

// print map, called by TIM2
void printMap () {
    if (mapOffset >= 59) {
        HAL_TIM_Base_Stop_IT(&htim2);
        return;
    }

    if (mario[1] >= 59 && isPlaying) {
        HAL_TIM_Base_Stop_IT(&htim2);
        gameWon();
        return;
    }

    for (int i = 0; i < 4; i++) {
        //skip the air
        if (i == 1 && mapOffset != 0 &&
        (mario[0] != 1 || !isPlaying)) {
            continue;
        }

        setCursor(0, i);
        write(cellTypeChar[BLADE]);

        // Mario hit the blades
        // Life Lost :(
        if (mario[1] == mapOffset - 1 && isPlaying) {
            lifeLost();
            return;
        }

        for (int j = mapOffset; j < mapOffset + 19; j++) {
            write(cellTypeChar[map[i][j]]);
        }
    }

    if (marioMode != JUMP && marioMode != RUMP)
        mapOffset++;
}

// set up game for play and start
void initGame(){
    // turn off leds
    HAL_GPIO_WritePin(GPIOE, 0xFFF00U, 0);

    // set all stats to default
    life = 3;
    coins = 0;
    mapOffset = 0;

    // turn on leds to indicate lives
    for (int i = 0; i < life ; i++){
        HAL_GPIO_WritePin(GPIOE, leds[i], 1);
    }

    // set up Mario
    mario[0] = 2;
    mario[1] = 4;
    map[2][4] = MARIO;

    srand(HAL_GetTick());
    generateMap();
    command(LCD_CLEARDISPLAY);

    printMap();
    collisionCheck(NONE);

    buttonCount = 1;
    isPlaying = 1;

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_UART_Receive_IT(&huart1, uartin, sizeof(uartin));
    HAL_TIM_Base_Start_IT(&htim2);
}

// Blue Button & Keypad handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t currTick = HAL_GetTick();
    // BIG BLUE BUTTON
    if (GPIO_Pin == GPIO_PIN_0)
    {
        if (buttonCount == 0) {
            initGame();
        }
        // generic play/pause
        else if (buttonCount == 1 && isPlaying) {
            isPlaying = 0;
            setCursor(5, 1);
            print("P A U S E D");
            unsigned char data[40];
            int sizestr = sprintf(data, "Game paused, press R to continue.\n");
            HAL_UART_Transmit(&huart1, data, sizestr, 1000);
            HAL_TIM_Base_Stop_IT(&htim2);
            HAL_TIM_Base_Stop_IT(&htim3);
            HAL_TIM_Base_Stop_IT(&htim4);
        }
        else {
            // clear pause graphics
            setCursor(5, 1);
            print ("           ");
            isPlaying = 1;
            HAL_TIM_Base_Start_IT(&htim4);
            HAL_TIM_Base_Start_IT(&htim2);
        }
    }
    // Keypad + debounce
    // (no need to check rows, all rows are similar!)
    else if (currTick - lastButtonTick >= 50) {
        // Run
        if (GPIO_Pin == kcols[0] && isPlaying){
            //runMario();
            marioMode = (marioMode != JUMP) ? RUN : RUMP;
            marioDir = getMoveDir(marioMode);
            collisionCheck((marioDir));
        }
        // Pause/Stop
        else if (GPIO_Pin == kcols[3] && isPlaying){
            isPlaying = 0;
            setCursor(5, 1);
            print("P A U S E D");
            unsigned char data[40];
            int sizestr = sprintf(data, "Game paused, press R to continue.\n");
            HAL_UART_Transmit(&huart1, data, sizestr, 1000);
            HAL_TIM_Base_Stop_IT(&htim2);
            HAL_TIM_Base_Stop_IT(&htim3);
            HAL_TIM_Base_Stop_IT(&htim4);
        }
        // Resume/Start
        else if (GPIO_Pin == kcols[2]){
            if(buttonCount == 0) {
                initGame();
            }
            else {
                // clear pause graphics
                setCursor(5, 1);
                print ("           ");
                isPlaying = 1;
                HAL_TIM_Base_Start_IT(&htim4);
                HAL_TIM_Base_Start_IT(&htim2);
            }
        }
        // Jump/space
        else if (GPIO_Pin == kcols[1] && isPlaying && jumpProg == -1){
            //Activate jump sequence
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_0);
            jumpProg = 0;
            //marioMode = (marioMode==RUN)? RUMP : JUMP;
            marioMode = JUMP;
            HAL_TIM_Base_Start_IT(&htim3);

        }
    }
    lastButtonTick = currTick;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Walk
        if((uartin[0] == 'd' || uartin[0] == 'D') && isPlaying){
            marioMode = (marioMode != JUMP) ? RUN : RUMP;
            //runMario();
            marioDir = getMoveDir(marioMode);
            collisionCheck((marioDir));
        }
        // Jump
        else if(uartin[0] == ' ' && isPlaying && jumpProg == -1){
            //Activate jump sequence
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_0);
            jumpProg = 0;
            //marioMode = JUMP;
            marioMode = (marioMode != RUN) ? JUMP : RUMP;
            HAL_TIM_Base_Start_IT(&htim3);
        }
        // Play/Resume/Start
        else if(uartin[0] == 'r' || uartin[0] == 'R'){
            if(buttonCount == 0) {
                isPlaying = 1;
                initGame();
            }
            else {
                setCursor(5, 1);
                print ("           ");isPlaying = 1;
                HAL_TIM_Base_Start_IT(&htim4);
                HAL_TIM_Base_Start_IT(&htim2);
            }
        }
        // Pause/Stop
        else if(uartin[0] == 'p' || uartin[0] == 'P' && isPlaying){
            isPlaying = 0;
            setCursor(5, 1);
            print("P A U S E D");
            unsigned char data[40];
            int sizestr = sprintf(data, "Game paused, press R to continue.\n");
            HAL_UART_Transmit(&huart1, data, sizestr, 1000);
            HAL_TIM_Base_Stop_IT(&htim2);
            HAL_TIM_Base_Stop_IT(&htim3);
            HAL_TIM_Base_Stop_IT(&htim4);
        }
        HAL_UART_Receive_IT(&huart1, uartin, sizeof(uartin));
    }
}

// print map (htim2) and print jump (htim3)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    // first check for speed changes, then print
    // TODO: which to print first? Mario/Map
	if (htim->Instance == TIM2){
	    // check if speed has changed
	    HAL_ADC_Start_IT(&hadc4);
        if(isPlaying) {
            marioDir = getMoveDir(marioMode);
            collisionCheck(marioDir);
        }
        // if mario is jumping, don't change offset
        if (marioMode != JUMP && marioMode != RUMP)
	        printMap();

        // lose immunity from start/life lost
        immunity--;
        if (immunity == 0){
            HAL_GPIO_WritePin(GPIOE, leds[6], 0);
        }
	}
	if (htim->Instance == TIM3){
        // do the jump routine
        jump();
	}
}

// volume to speed
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // use Volume module to control map ~~Speed~~
    if (hadc->Instance == ADC4)
    {
        uint32_t x = HAL_ADC_GetValue(&hadc4);
        double lastVolSpeed = volSpeed;
        volSpeed = 0.5 + (x * 1.5 / 4095);
        if (volSpeed <= lastVolSpeed - 0.35
            || volSpeed >= lastVolSpeed + 0.35)
        {
            HAL_TIM_Base_Stop_IT(&htim2);
            htim2.Init.Period = (uint32_t) (10000 * (1/volSpeed));
            unsigned char data[15];
            int sizestr = sprintf(data, "%.2lf speed\n", volSpeed);
            HAL_UART_Transmit(&huart1, data, sizestr, 1000);
            //__HAL_TIM_SET_AUTORELOAD(&htim2,volSpeed);
            HAL_TIM_Base_Start_IT(&htim2);
        }
    }
    HAL_ADC_Stop_IT(&hadc4);
}

// get direction of move based on move mode
enum moveDir getMoveDir(enum moveType move){
    switch (move){
        case JUMP:
            // in 0 & 1 we're going up
            if (jumpProg != -1 && jumpProg < 2){
                return UP;
            }
            // in 2 & 3 it's down
            // (back to cells 1 & 2)
            else if (jumpProg != -1){
                return DOWN;
            }
            break;
        case RUMP:
            if (jumpProg != -1 && jumpProg < 2){
                return UPRIGHT;
            }
            else if (jumpProg != -1){
                return DOWNRIGHT;
            }
            return RIGHT;
            break;
        case RUN:
            return RIGHT;
        case HALT:
        default:
            return NONE;
    }
    return NONE;
}

// collision detection and handling
void collisionCheck(enum moveDir dir) {
    int oldy = mario[0];
    int oldx = mario[1];

    // MARIO WON!
    if (mario[1] >= 59){
        HAL_TIM_Base_Stop_IT(&htim2);
        // bye mario!
        map[oldy][oldx] = genmap[oldy][oldx];
        gameWon();
        return;
    }

    switch (dir){
        //FIXME: handle upright & downright
        case RIGHT:
        case DOWNRIGHT:
        case UPRIGHT:
            // mario is in last visible cell,
            // move window
            if (mario[1] >= mapOffset + 18){
                mapOffset += 3;
            }
            // path is clear
            if (map[oldy][oldx + 1] == AIR){
                // reset old cell to original state
                map[oldy][oldx] = genmap[oldy][oldx];
                map[oldy][oldx+1] = MARIO;
                mario[1] = oldx + 1;
            }
            // fall back to ground if no pipe
            if (oldy == 1 && map[2][oldx] != PIPE && jumpProg > 1){
                map[1][oldx] = genmap[1][oldx];
                map[2][oldx] = MARIO;
                mario[0] = 2;
                setCursor(oldx - mapOffset + 2, 1);
                write(cellTypeChar[map[1][oldx]]);
                setCursor(mario[1] - mapOffset + 2, mario[0]);
                write(cellTypeChar[MARIO]);
            }
            //TODO: check for pits, here??
            break;
        case UP:
            // mario can *still* go up
            if (mario[0] > 0){
                // TODO: check ordering of conditions
                // space to move / no collision
                if (map[oldy-1][oldx] == AIR ){
                    // reset old cell to original state
                    map[oldy][oldx] = genmap[oldy][oldx];
                    map[oldy-1][oldx] = MARIO;
                    mario[0] = oldy - 1;
                }
                // coins $_$
                else if (oldy == 1 && map[oldy-1][oldx] == CHEST){
                    // break the chest
                    map[0][oldx] = AIR;
                    genmap[0][oldx] = AIR;
                    //map[oldy-1][oldx] = MARIO;
                    // add coins to mario
                    // (random amount between 1-3)
                    int c = rand() % 3 + 1;
                    coins += c;
                    char data[45];
                    int sizestr = sprintf(data, "You found %d coins $_$\t"
                                                "Total coins: %d\n", c, coins);
                    HAL_UART_Transmit(&huart1, data, sizestr, 1000);
                    setCursor(oldx - mapOffset + 2, 0);
                    write(cellTypeChar[AIR]);
                    break;
                }
            }
            setCursor(oldx - mapOffset + 2, oldy);
            write(cellTypeChar[map[oldy][oldx]]);
            setCursor(mario[1] - mapOffset + 2, mario[0]);
            write(cellTypeChar[MARIO]);
            break;
        case DOWN:
            // if there's *still* space to go down
            // not pits!
            if (mario[0] < 2 && map[oldy+1][oldx] == AIR){
                map[oldy][oldx] = genmap[oldy][oldx];
                map[oldy+1][oldx] = MARIO;
                mario[0] = oldy + 1;
            }
            setCursor(oldx - mapOffset + 2, oldy);
            write(cellTypeChar[map[oldy][oldx]]);
            setCursor(mario[1] - mapOffset + 2, mario[0]);
            write(cellTypeChar[MARIO]);
            break;
        case NONE:
        default:
            // fall back to ground if no pipe
            if (oldy == 1 && map[2][oldx] != PIPE){
                map[1][oldx] = genmap[oldy][oldx];
                map[2][oldx] = MARIO;
                mario[0] = oldy + 1;
                setCursor(oldx - mapOffset + 2, oldy);
                write(cellTypeChar[map[oldy][oldx]]);
                setCursor(mario[1] - mapOffset + 2, mario[0]);
                write(cellTypeChar[MARIO]);
            }
            break;
    }

    // fall into pit :(
    if (mario[0] == 2 && genmap[3][mario[1]] == AIR){
        // handle life loss & send stats
        unsigned char data[40];
        int sizestr = sprintf(data, "That was a steep fall :( \n");
        HAL_UART_Transmit(&huart1, data, sizestr, 1000);
        HAL_GPIO_WritePin(GPIOE, leds[6], 1);
        lifeLost();
        // reset the map to original state,
        // move mario ahead (no pipes/pits next to pits)
        map[mario[0]][mario[1]] = genmap[mario[0]][mario[1]];
        mario[1] += 1;
        return;
    }

    ////walk on pipes
    //if (map[2][x] == PIPE && oldy == 0
    //&& (dir == RIGHT || dir == DOWN || dir == NONE)){
    //    mario[0] = 1;
    //    //newy = 1;
    //    printMario(newy, newx, 0);
    //    return;
    //}

    //// down to earth
    //if (map[2][x] == AIR && oldy == 1
    //&& (dir != UP)){
    //    //mario[0] = 2;
    //    newy = 2;
    //    printMario(newy, newx, 0);
    //    return;
    //}
    marioMode = (marioMode == RUMP || marioMode == JUMP)? JUMP : HALT;
}

// walk sequence handler + block/pipe handler while walking
void runMario (){
    // don't move if there's a block or pipe
    if((mario[0] == 2 && map[2][mario[1]+1] == PIPE)
       || (mario[0] == 0 && map[0][mario[1]+1] == BLOCK)
       || (mario[0] == 0 && map[0][mario[1]+1] == CHEST)) {
        unsigned char data[40];
        int size = sprintf(data, "BLOCK? PIPE?\n");
        HAL_UART_Transmit(&huart1, data, size,1000);
        return;
    }

    // falling is only possible when not jumping
    if ((mario[0] == 2 && map[3][mario[1]+1] == AIR)
        && marioMode == RUN){
        unsigned char data[40];
        int sizestr = sprintf(data, "That was a steep fall :( \n");
        HAL_UART_Transmit(&huart1, data, sizestr, 1000);
        HAL_GPIO_WritePin(GPIOE, leds[6], 1);
        lifeLost();
        int x = mario[1];
        mario[1]  += 2;
        return;
    }

    int x = mario[1];
    HAL_TIM_Base_Stop_IT(&htim2);
    // if got close to map edge, scroll
    if (x >= mapOffset + 19){
        mapOffset += 3;
        //TODO: CHECK THIS
        printMap();
    }

    x++;
    mario[1] = x;
    HAL_TIM_Base_Start_IT(&htim2);
    // if RUMP, switch back to JUMP mode
    marioMode = (marioMode == RUN) ? HALT : JUMP;
}

// jump sequence handler
void jump (){
    switch (jumpProg){
        case 3:
            marioDir = getMoveDir(marioMode);
            collisionCheck((marioDir));
            marioMode = HALT;
            jumpProg = -1;
            // relay pass
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_0);
            HAL_TIM_Base_Stop_IT(&htim3);
            break;
        case 1:
        case 0:
            marioDir = getMoveDir(marioMode);
            collisionCheck((marioDir));
            break;
        case 2:
        default:
            // coins $_$
            //if (map[0][mario[1]] == CHEST) {
            //    // break the chest
            //    map[0][oldx] = AIR;
            //    charMap[0][oldx] = cellTypeChar[AIR];
            //    // add coins to mario (random between 1-3)
            //    int c = rand() % 3 + 1;
            //    char data[45] = "";
            //    coins += c;
            //    int sizestr = sprintf(data, "You found %d coins $_$\t"
            //                                "Total coins: %d\n", c, coins);
            //    HAL_UART_Transmit(&huart1, data, sizestr, 1000);
            //}
            marioDir = getMoveDir(marioMode);
            collisionCheck((marioDir));
            break;
    }

    if (marioMode == RUMP)
        marioMode = (jumpProg == -1)? HALT : JUMP;
    if (jumpProg != -1)
        jumpProg += 1;
}

//show splash screen at start (on lcd)
void splash (){
    noCursor();
    unsigned char data[40] = "";
    int sizestr = sprintf(data, "Press the blue button to start.\n");
    HAL_UART_Transmit(&huart1, data, sizestr, 1000);
    sizestr = sprintf(data, "Use 'D' to go forward, space to jump.\n");
    HAL_UART_Transmit(&huart1, data, sizestr, 1000);
    // turn on everything
    HAL_GPIO_WritePin(GPIOE, 0xFFF00U, 1);
    setCursor(1,0);
    for (int i=0; i<18; i++) {
        write(cellTypeChar[4 + (i % 2)]);
    }
    setCursor(0,1);
    print("M I C R O  M A R I O");
    setCursor(1,3);
    for (int i=0; i<18; i++){
        write(cellTypeChar[1]);
    }

}

// life lost handler (can call game lost)
void lifeLost (){
    // not quite yet!
    if(immunity > 0 || life <0)
        return;
    //switch off a life
    HAL_GPIO_WritePin(GPIOE, leds[life-1], 0);
    life--;
    if (life == 0){
        gameLost();
        return;
    }
    else {
        HAL_TIM_Base_Stop_IT(&htim2);
        unsigned char data[30] = "";
        int sizestr = sprintf(data, "Life lost, remaining %d.\n", life);
        HAL_UART_Transmit(&huart1, data, sizestr, 1000);
        // take a breath
        immunity = 3;
        marioMode = HALT;
        HAL_UART_Receive_IT(&huart1, uartin, sizeof(uartin));
        HAL_TIM_Base_Start_IT(&htim2);
    }
}

// game lost handler, can restart
void gameLost (){
    HAL_TIM_Base_Stop_IT(&htim2);
    command(LCD_CLEARDISPLAY);

    //print pretty stuff and stats (to uart & lcd)
    unsigned char data[40] = "";
    int sizestr;
    sizestr = sprintf(data, "\nYou Lost X_X\n");
    HAL_UART_Transmit(&huart1, data, sizestr, 1000);
    sizestr = sprintf(data, "$$  Coins: %d  $$\n", coins);
    HAL_UART_Transmit(&huart1, data, sizestr, 1000);
    // turn on all leds
    HAL_GPIO_WritePin(GPIOE, 0xFFF00U, 1);
    setCursor(1,0);

    for (int i=0; i<5; i++) {
        write(cellTypeChar[BLOCK]);
    }
    sprintf(data, "   %2d   ", coins);
    print(data);

    for (int i=0; i<5; i++) {
        write(cellTypeChar[BLOCK]);
    }

    setCursor(3,1);
    print("Y O U  L O S T");
    setCursor(8,2);
    print("X_X");
    setCursor(1,3);

    for (int i=0; i<18; i++){
        write(cellTypeChar[GROUND]);
    }

    HAL_Delay(4000);
    // restart stuff
    isPlaying = 0;
    buttonCount = 0;
    HAL_TIM_Base_Start_IT(&htim2);
}


// game won handler
void gameWon (){
    HAL_TIM_Base_Stop_IT(&htim2);
    command(LCD_CLEARDISPLAY);

    unsigned char data[40] = "";
    int sizestr;
    sizestr = sprintf(data, "\nYou Won\n");
    HAL_UART_Transmit(&huart1, data, sizestr, 1000);
    sizestr = sprintf(data, "$$   Coins: %d   $$\n", coins);
    HAL_UART_Transmit(&huart1, data, sizestr, 1000);
    // turn on all leds
    HAL_GPIO_WritePin(GPIOE, 0xFFF00U, 1);
    setCursor(1,0);

    for (int i=0; i<5; i++) {
        write(cellTypeChar[BLOCK]);
    }
    sprintf(data, "   %2d   ", coins);
    print(data);

    for (int i=0; i<5; i++) {
        write(cellTypeChar[BLOCK]);
    }

    setCursor(3,1);
    print("Y O U  W O N !");
    setCursor(8,2);
    print("*_*");
    setCursor(1,3);

    for (int i=0; i<18; i++){
        write(cellTypeChar[GROUND]);
    }

    HAL_Delay(4000);
    // restart stuff
    isPlaying = 0;
    buttonCount = 0;
    HAL_TIM_Base_Start_IT(&htim2);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
