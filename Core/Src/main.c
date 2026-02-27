/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>

#include "timers.h"

#include "appiface.h"
#include "avoltage.h"
#include "sim800l.h"
#include "watchdog.h"
#include "ptasks.h"
#include "button.h"
#include "siface.h"
#include "logger.h"
#include "params.h"
#include "atomic.h"
#include "mqueue.h"
#include "w25q_s.h"
#include "as5600.h"
#include "aht20.h"
#include "ota.h"
#include "fws.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART_BUFFER_SIZE \
	(SIM800L_UART_BUFFER_SIZE > SIFACE_UART_BUFFER_SIZE ? \
	SIM800L_UART_BUFFER_SIZE : SIFACE_UART_BUFFER_SIZE)

#define STACK_COLOR_WORD 0xACACACAC

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for def */
osThreadId_t defHandle;
const osThreadAttr_t def_attributes = {
  .name = "def",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

volatile struct bl_params bl __attribute__((section(".noinit")));

StaticTimer_t hz_timer_buf;
TimerHandle_t hz_timer_handle;

volatile uint32_t timestamp;

uint8_t ub_mod[UART_BUFFER_SIZE];
uint8_t ub_sif[UART_BUFFER_SIZE];

params_t params;
struct button btn;
struct siface siface;
#ifdef LOGGER
struct logger logger;
#endif
struct w25q_s mem;
struct sim800l mod;
struct ota ota;
struct aht20 aht;
struct as5600 pot;
struct counter cnt;
struct avoltage avlt;
struct appiface appif;

struct actual actual;
struct sensors sens;
struct ecounter ecnt;
struct app app;
struct system sys;

volatile int init_done = 0;

extern const uint32_t *_ebss;
extern const uint32_t *_estack;
extern const uint32_t *_app;
#define APP_ADDRESS ((uint32_t) &_app)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART2_UART_Init(void);
void task_default(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void usb_cdc_rx_callback(uint8_t* buf, size_t size)
{
	siface_rx_irq(&siface, (const char *) buf, size);
}

void usb_cdc_tx_callback(void)
{
	siface_tx_irq(&siface);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
		return;

//	if (huart == &huart1)
//	{
//		siface_rx_irq(&siface, (const char *) ub_sif, size);
//		HAL_UARTEx_ReceiveToIdle_DMA(huart, ub_sif, UART_BUFFER_SIZE);
//	}
	if (huart == &huart2)
	{
		sim800l_irq(&mod, (const char *) ub_mod, size);
		HAL_UARTEx_ReceiveToIdle_DMA(huart, ub_mod, UART_BUFFER_SIZE);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
//	if (huart == &huart1)
//	{
//		HAL_UARTEx_ReceiveToIdle_DMA(huart, ub_sif, UART_BUFFER_SIZE);
//	}
	if (huart == &huart2)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(huart, ub_mod, UART_BUFFER_SIZE);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
//	if (huart == &huart1)
//	{
//		siface_tx_irq(&siface);
//	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (!init_done)
		return;

	if (GPIO_Pin == EXTI0_HALL_Pin)
	{
		counter_irq(&cnt);
	}
}

void hz_callback(TimerHandle_t timer)
{
	atomic_inc(&timestamp);
}

void btn_callback(void)
{
	if (!init_done)
		return;

	task_sensors_notify(&sens);
}

//
static void stack_color(void)
{
	volatile uint32_t *p, *end;

	p = (uint32_t *) &_ebss;
	end = (uint32_t *) __get_MSP();
	p++;

	while (p < end)
	{
		*p = STACK_COLOR_WORD;
		p++;
	}
}

//
static size_t stack_size(void)
{
	volatile uint32_t *p, *end;

	p = (uint32_t *) &_ebss;
	end = (uint32_t *) &_estack;
	p++;

	while ((p < end) && (*p == STACK_COLOR_WORD))
		p++;

	return (size_t) end - (size_t) p;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  SCB->VTOR = APP_ADDRESS;
  __enable_irq();

  //
  stack_color();

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
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_IWDG_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Deinitialization
  HAL_I2C_DeInit(&hi2c2);

  //
  //DWT_cnt_init();

  //
  params_init();
  params_get(&params);

  /* todo */
  // SIM800L power
  HAL_GPIO_WritePin(MDM_EN_PRE_GPIO_Port, MDM_EN_PRE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MDM_EN_GPIO_Port, MDM_EN_Pin, GPIO_PIN_SET);

  //
  memset(&actual, 0, sizeof(actual));
  actual.mutex = xSemaphoreCreateMutex();

  //
  appif.timestamp = &timestamp;
  appif.params = &params;
  appif.actual = &actual;
  appif.bl = &bl;
  memcpy(&appif.uparams, &params, sizeof(params));

  //
  button_init(&btn, BTN_MB_GPIO_Port, BTN_MB_Pin, btn_callback);
  siface_init(&siface, 32, appiface, &appif);
#ifdef LOGGER
  logger_init(&logger, &siface);
#endif
  w25q_s_init(&mem, &hspi2, SPI2_CS_GPIO_Port, SPI2_CS_Pin);
  sim800l_init(&mod, &huart2, MDM_RST_GPIO_Port, MDM_RST_Pin, params.apn);
  ota_init(&ota, &mod, &mem, params.secret, params.url_ota);
  as5600_init(&pot, &hi2c1, MX_I2C1_Init, SENS_EN_GPIO_Port,
		  SENS_EN_Pin /* 0x6C */);
  aht20_init(&aht, &hi2c2, MX_I2C2_Init, AHT20_EN_GPIO_Port,
		  AHT20_EN_Pin /* 0x70 */);
  counter_init(&cnt, HALL_EN_GPIO_Port, HALL_EN_Pin);
  avoltage_init(&avlt, &hadc1, 2);

  //
  mqueue_init(&mem);

  // sens
  memset(&sens, 0, sizeof(sens));
  sens.queue = mqueue_create(SENSORS_QUEUE_SECNUM,
		  sizeof(struct sensor_record));
  sens.avlt = &avlt;
  sens.pot = &pot;
  sens.aht = &aht;
  sens.cnt = &cnt;
  sens.timestamp = &timestamp;
  sens.params = &params;
  sens.actual = &actual;
  sens.events = 0;

  // ecnt
  memset(&ecnt, 0, sizeof(ecnt));
  ecnt.cnt = &cnt;
  ecnt.params = &params;
  ecnt.actual = &actual;

  // app
  memset(&app, 0, sizeof(app));
  app.timestamp = &timestamp;
  app.params = &params;
  app.sens = &sens;
  app.mod = &mod;
  app.bl = &bl;

  // sys
  memset(&sys, 0, sizeof(sys));
  sys.params = &params;
  sys.bl = &bl;

  // wd
  watchdog_init(&hiwdg, EXT_WDG_GPIO_Port, EXT_WDG_Pin);

  //
  /* todo: replace with USB */
  //HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ub_sif, UART_BUFFER_SIZE);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, ub_mod, UART_BUFFER_SIZE);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  hz_timer_handle = xTimerCreateStatic("hz", pdMS_TO_TICKS(1000), pdTRUE,
		  (void *) 0, hz_callback, &hz_timer_buf);
  xTimerStart(hz_timer_handle, 0);

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of def */
  defHandle = osThreadNew(task_default, NULL, &def_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  task_watchdog();
	task_logging(&sys);
  task_siface(&siface);
  task_sensors(&sens);
  task_ecounter(&ecnt);
  task_sim800l(&mod);
  task_button(&btn);
  task_ota(&ota);
  task_app(&app);

  // For GPIO interrupts and button (?)
  init_done = 1;

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */

  //
  sys.main_stack_size = stack_size();

  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_DB_GPIO_Port, LED_DB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HALL_EN_Pin|MDM_EN_Pin|EXT_WDG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AHT20_EN_Pin|SENS_EN_Pin|MDM_EN_PRE_Pin|LED_MB_Pin
                          |MDM_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_DB_Pin */
  GPIO_InitStruct.Pin = LED_DB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_DB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXTI0_HALL_Pin */
  GPIO_InitStruct.Pin = EXTI0_HALL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXTI0_HALL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_MB_Pin CHRG_OK_Pin CHRG_STAT_Pin */
  GPIO_InitStruct.Pin = BTN_MB_Pin|CHRG_OK_Pin|CHRG_STAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : HALL_EN_Pin MDM_EN_Pin EXT_WDG_Pin */
  GPIO_InitStruct.Pin = HALL_EN_Pin|MDM_EN_Pin|EXT_WDG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AHT20_EN_Pin SENS_EN_Pin SPI2_CS_Pin MDM_EN_PRE_Pin
                           LED_MB_Pin MDM_RST_Pin */
  GPIO_InitStruct.Pin = AHT20_EN_Pin|SENS_EN_Pin|SPI2_CS_Pin|MDM_EN_PRE_Pin
                          |LED_MB_Pin|MDM_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_task_default */
/**
  * @brief  Function implementing the def thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_task_default */
void task_default(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */

  /* todo: move USB init */

  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_DB_GPIO_Port, LED_DB_Pin);
    osDelay(200);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
