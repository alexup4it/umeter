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

#include "task.h"
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

/* RTC wakeup timer uses LSI/2 = ~16 kHz clock */
#define RTC_WKUP_CLK_HZ          16000U
/* Maximum sleep time limited by external WDG (1.6 s) */
#define MAX_SLEEP_MS             1000U

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
  .stack_size = 96 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

volatile struct bl_params bl __attribute__((section(".noinit")));

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

EventGroupHandle_t sync_events;
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



void btn_callback(void)
{
	if (!init_done)
		return;

	task_sensors_notify(&sens);
}

void sim800l_hw_init_cb(void)
{
	MX_USART2_UART_Init();
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, ub_mod, UART_BUFFER_SIZE);
}

/*---------------------------------------------------------------------------*/
/* RTC wakeup timer — direct register access (no HAL RTC driver needed)      */
/*---------------------------------------------------------------------------*/

static inline void rtc_unlock(void)
{
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
}

static inline void rtc_lock(void)
{
	RTC->WPR = 0xFF;
}

static void rtc_wakeup_init(void)
{
	/* Enable PWR clock and backup domain access */
	__HAL_RCC_PWR_CLK_ENABLE();
	PWR->CR |= PWR_CR_DBP;

	/* Select LSI as RTC clock source and enable RTC */
	if ((RCC->BDCR & RCC_BDCR_RTCSEL) != RCC_BDCR_RTCSEL_1)
	{
		/* Reset backup domain to change RTC source */
		RCC->BDCR |= RCC_BDCR_BDRST;
		RCC->BDCR &= ~RCC_BDCR_BDRST;
		/* LSI selected (RTCSEL = 10) */
		RCC->BDCR |= RCC_BDCR_RTCSEL_1;
	}
	RCC->BDCR |= RCC_BDCR_RTCEN;

	/* Unlock RTC registers */
	rtc_unlock();

	/* Enter init mode */
	RTC->ISR |= RTC_ISR_INIT;
	while (!(RTC->ISR & RTC_ISR_INITF))
		;

	/* Set prescalers: AsynchPrediv=127, SynchPrediv=249 (irrelevant for wakeup,
	 * but needed for valid init) */
	RTC->PRER = (127U << 16) | 249U;

	/* Exit init mode */
	RTC->ISR &= ~RTC_ISR_INIT;

	/* Disable wakeup timer to configure it */
	RTC->CR &= ~RTC_CR_WUTE;
	while (!(RTC->ISR & RTC_ISR_WUTWF))
		;

	/* Select clock: RTCCLK/2 (LSI/2 ≈ 16 kHz) → WUCKSEL = 011 */
	RTC->CR = (RTC->CR & ~RTC_CR_WUCKSEL) | RTC_CR_WUCKSEL_0 | RTC_CR_WUCKSEL_1;

	/* Enable wakeup interrupt */
	RTC->CR |= RTC_CR_WUTIE;

	/* Clear wakeup flag */
	RTC->ISR &= ~RTC_ISR_WUTF;

	rtc_lock();

	/* Configure EXTI line 22 (RTC wakeup) — rising edge, interrupt mode */
	EXTI->IMR |= EXTI_IMR_MR22;
	EXTI->RTSR |= EXTI_RTSR_TR22;
	EXTI->PR = EXTI_PR_PR22; /* Clear pending */

	/* Enable RTC_WKUP IRQ in NVIC */
	HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
}

static void rtc_wakeup_set(uint32_t ticks_ms)
{
	uint32_t wut;

	if (ticks_ms > MAX_SLEEP_MS)
		ticks_ms = MAX_SLEEP_MS;

	/* Calculate wakeup timer value: WUT = (ms * RTC_WKUP_CLK_HZ) / 1000 - 1 */
	wut = (ticks_ms * RTC_WKUP_CLK_HZ) / 1000U;
	if (wut > 0)
		wut--;
	if (wut > 0xFFFF)
		wut = 0xFFFF;

	rtc_unlock();

	/* Disable wakeup timer */
	RTC->CR &= ~RTC_CR_WUTE;
	while (!(RTC->ISR & RTC_ISR_WUTWF))
		;

	/* Set wakeup auto-reload value */
	RTC->WUTR = wut;

	/* Clear wakeup flag and EXTI pending */
	RTC->ISR &= ~RTC_ISR_WUTF;
	EXTI->PR = EXTI_PR_PR22;

	/* Enable wakeup timer */
	RTC->CR |= RTC_CR_WUTE;

	rtc_lock();
}

static void rtc_wakeup_stop(void)
{
	rtc_unlock();
	RTC->CR &= ~RTC_CR_WUTE;
	RTC->ISR &= ~RTC_ISR_WUTF;
	rtc_lock();
	EXTI->PR = EXTI_PR_PR22;
}

void RTC_WKUP_IRQHandler(void)
{
	if (RTC->ISR & RTC_ISR_WUTF)
	{
		rtc_unlock();
		RTC->ISR &= ~RTC_ISR_WUTF;
		rtc_lock();
	}
	EXTI->PR = EXTI_PR_PR22;
}

/*---------------------------------------------------------------------------*/
/* Low-power GPIO helpers for Stop mode                                      */
/*---------------------------------------------------------------------------*/

/*
 * Before entering Stop mode, reconfigure all floating / AF pins to analog
 * to eliminate leakage through external pull-ups, flash bus, etc.
 * Analog mode gives the lowest possible GPIO current (no input Schmitt
 * trigger, no pull, no AF driver).
 *
 * Pins that MUST keep their state (output-driven power-enable, CS, etc.)
 * are left untouched.
 */
static void gpio_enter_stop(void)
{
	GPIO_InitTypeDef g = {0};
	g.Mode  = GPIO_MODE_ANALOG;
	g.Pull  = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;

	/* --- W25Q flash: deep power-down BEFORE disabling SPI2 ---
	 * Reduces flash standby from ~25 µA to ~1 µA.
	 * SPI2 must still be functional at this point. */
	w25q_hw_deinit(&mem.mem);

	/* --- SPI2 bus (PB13-SCK, PB14-MISO, PB15-MOSI) ---
	 * Left in AF_PP after MX_SPI2_Init → leaks into W25Q flash.
	 * CS (PB12) stays high (output) to keep flash deselected. */
	g.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	HAL_GPIO_Init(GPIOB, &g);

	/* --- I2C1 (PB6-SCL, PB7-SDA) ---
	 * DeInit leaves them floating; external pull-ups → ~0.7 mA each. */
	g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	HAL_GPIO_Init(GPIOB, &g);

	/* --- I2C2 (PB10-SCL, PB3-SDA) ---
	 * Same pull-up issue. */
	g.Pin = GPIO_PIN_3 | GPIO_PIN_10;
	HAL_GPIO_Init(GPIOB, &g);

	/* --- USART2 (PA2-TX, PA3-RX) --- already DeInit when modem off,
	 * but HAL_GPIO_DeInit only resets to input-float → set analog. */
	g.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	HAL_GPIO_Init(GPIOA, &g);

	/* --- EXTI0 Hall sensor input (PA0) --- already masked in EXTI,
	 * but the pin itself as input-float picks up noise → analog. */
	g.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOA, &g);

	/* --- ADC1: disable peripheral to cut analog bias current --- */
	__HAL_RCC_ADC1_CLK_DISABLE();

	/* --- Disable DMA1 clock (no active transfers during Stop) --- */
	__HAL_RCC_DMA1_CLK_DISABLE();

	/* --- Disable SPI2 clock --- */
	__HAL_RCC_SPI2_CLK_DISABLE();

	/* --- Disable GPIOH clock (only HSE pins, unused in Stop) --- */
	__HAL_RCC_GPIOH_CLK_DISABLE();
}

/*
 * After waking from Stop, restore peripheral pins and clocks.
 * SystemClock_Config() has already been called at this point.
 */
static void gpio_exit_stop(void)
{
	GPIO_InitTypeDef g = {0};

	/* Re-enable port clocks that were disabled */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_SPI2_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();

	/* --- Restore SPI2 pins to AF5 --- */
	g.Pin       = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	g.Mode      = GPIO_MODE_AF_PP;
	g.Pull      = GPIO_NOPULL;
	g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	g.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &g);

	/* --- W25Q flash: release deep power-down AFTER SPI2 is restored --- */
	w25q_hw_init(&mem.mem);

	/* --- Restore EXTI0 (Hall sensor) as falling-edge interrupt --- */
	g.Pin  = GPIO_PIN_0;
	g.Mode = GPIO_MODE_IT_FALLING;
	g.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(EXTI0_HALL_GPIO_Port, &g);

	/* I2C1/I2C2 and USART2 pins are restored lazily by their hw_init
	 * callbacks (as5600, aht20, sim800l) when the peripheral is
	 * actually needed — no need to restore here. */
}

/*---------------------------------------------------------------------------*/
/* Custom tickless idle: RTC wakeup + Stop mode                              */
/*---------------------------------------------------------------------------*/

void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
	uint32_t sleep_ms;

	/* Cap at MAX_SLEEP_MS for external WDG safety */
	if (xExpectedIdleTime > pdMS_TO_TICKS(MAX_SLEEP_MS))
		xExpectedIdleTime = pdMS_TO_TICKS(MAX_SLEEP_MS);

	sleep_ms = xExpectedIdleTime * (1000U / configTICK_RATE_HZ);

	/* Enter critical section (disable interrupts) */
	__asm volatile("cpsid i" ::: "memory");
	__asm volatile("dsb");
	__asm volatile("isb");

	/* Confirm sleep is still valid */
	if (eTaskConfirmSleepModeStatus() == eAbortSleep)
	{
		__asm volatile("cpsie i" ::: "memory");
		return;
	}

	/* Stop mode is unsafe when:
	 *  - USB OTG FS is active (clocks stop → host sees disconnect)
	 *  - SIM800L modem is on (UART DMA resumes at wrong baud on HSI wakeup)
	 * Fallback to regular Sleep (WFI) — SysTick keeps running.
	 */
	if ((RCC->AHB2ENR & RCC_AHB2ENR_OTGFSEN) ||
		HAL_GPIO_ReadPin(MDM_EN_GPIO_Port, MDM_EN_Pin) == GPIO_PIN_SET)
	{
		__asm volatile("cpsie i" ::: "memory");
		__WFI();
		return;
	}

	/* Refresh watchdog before entering Stop mode */
	watchdog_reset();

	/* Suspend HAL tick (TIM1) */
	HAL_SuspendTick();

	/* Mask EXTI0 (Hall sensor) — PA0 floats when sensor is off,
	 * noise generates spurious falling edges that wake MCU instantly */
	EXTI->IMR &= ~EXTI_IMR_MR0;

	/* Reconfigure GPIO/peripherals for minimal leakage */
	gpio_enter_stop();

	/* Set RTC wakeup timer */
	rtc_wakeup_set(sleep_ms);

	/* --- Enter Stop mode (low-power regulator) --- */
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	/* --- Woken up (interrupts still disabled) --- */

	/* Check if RTC wakeup timer fired (full sleep) before clearing */
	uint32_t wutf = RTC->ISR & RTC_ISR_WUTF;

	/* Stop the RTC wakeup timer */
	rtc_wakeup_stop();

	/* Restore system clock (HSE + PLL) after Stop mode */
	SystemClock_Config();

	/* Restore GPIO/peripherals after wakeup */
	gpio_exit_stop();

	/* Resume HAL tick */
	HAL_ResumeTick();

	/* Restore EXTI0 (Hall sensor interrupt) */
	EXTI->IMR |= EXTI_IMR_MR0;

	/* Re-enable interrupts */
	__asm volatile("cpsie i" ::: "memory");

	/* Advance the RTOS tick count:
	 * If RTC WUTF fired — full sleep elapsed.
	 * If woken by something else — conservative 1 tick. */
	vTaskStepTick(wutf ? xExpectedIdleTime : 1);
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

  /* SIM800L power is managed by sim800l module (2-stage: EN_PRE -> EN) */

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
  sim800l_init(&mod, &huart2, sim800l_hw_init_cb,
		  MDM_RST_GPIO_Port, MDM_RST_Pin,
		  MDM_EN_GPIO_Port, MDM_EN_Pin,
		  MDM_EN_PRE_GPIO_Port, MDM_EN_PRE_Pin,
		  params.apn);
  ota_init(&ota, &mod, &mem, params.secret, params.url_ota);
  as5600_init(&pot, &hi2c1, MX_I2C1_Init, SENS_EN_GPIO_Port,
		  SENS_EN_Pin /* 0x6C */);
  aht20_init(&aht, &hi2c2, MX_I2C2_Init, AHT20_EN_GPIO_Port,
		  AHT20_EN_Pin /* 0x70 */);
  counter_init(&cnt, HALL_EN_GPIO_Port, HALL_EN_Pin);
  avoltage_init(&avlt, &hadc1, 2, VBAT_MEAS_EN_GPIO_Port, VBAT_MEAS_EN_Pin);

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

  // RTC wakeup timer for Stop mode
  rtc_wakeup_init();

  // Flash power-down in Stop mode
  PWR->CR |= PWR_CR_FPDS;

  //
  /* todo: replace with USB */
  //HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ub_sif, UART_BUFFER_SIZE);
  /* UART2 DMA RX is now started inside sim800l_power_on via hw_init callback */

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

  sync_events = xEventGroupCreate();

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
  task_blink();
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
  HAL_GPIO_WritePin(GPIOA, HALL_EN_Pin|MDM_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AHT20_EN_Pin|SENS_EN_Pin|VBAT_MEAS_EN_Pin|MDM_EN_PRE_Pin|LED_MB_Pin
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

  /*Configure GPIO pins : HALL_EN_Pin MDM_EN_Pin */
  GPIO_InitStruct.Pin = HALL_EN_Pin|MDM_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT_WDG_Pin (Hi-Z, no pull) */
  GPIO_InitStruct.Pin = EXT_WDG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AHT20_EN_Pin SENS_EN_Pin VBAT_MEAS_EN_Pin SPI2_CS_Pin
                           MDM_EN_PRE_Pin LED_MB_Pin MDM_RST_Pin */
  GPIO_InitStruct.Pin = AHT20_EN_Pin|SENS_EN_Pin|VBAT_MEAS_EN_Pin|SPI2_CS_Pin
                          |MDM_EN_PRE_Pin|LED_MB_Pin|MDM_RST_Pin;
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
  uint32_t sync_sec = 0;
  TickType_t wake = xTaskGetTickCount();
  EventBits_t bits;

  for(;;)
  {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000));
    atomic_inc(&timestamp);
    sync_sec++;
    bits = 0;

    if (params.mtime_count && (sync_sec % params.mtime_count == 0))
      bits |= SYNC_BIT_ECOUNTER;

    if (params.period_sen && sync_sec >= 5 &&
        ((sync_sec - 5) % params.period_sen == 0))
      bits |= SYNC_BIT_SENSORS;

    /* APP fires 10 seconds after the aligned boundary so that sensors
     * have time to read and write fresh data before app reads the queue */
    if (params.period_app && sync_sec >= 10 &&
        ((sync_sec - 10) % params.period_app == 0))
      bits |= SYNC_BIT_APP;

    if (sync_sec % 5 == 0)
      bits |= SYNC_BIT_BLINK;

    bits |= SYNC_BIT_WATCHDOG;

    xEventGroupSetBits(sync_events, bits);
  }
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
