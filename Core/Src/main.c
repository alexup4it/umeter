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
#include "dma.h"
#include "gpio.h"
#include "iwdg.h"
#include "rtc.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>

#include "adc.h"
#include "aht20.h"
#include "appiface.h"
#include "as5600.h"
#include "atomic.h"
#include "avoltage.h"
#include "button.h"
#include "fws.h"
#include "i2c.h"
#include "logger.h"
#include "mqueue.h"
#include "ota.h"
#include "params.h"
#include "ptasks.h"
#include "rtctime.h"
#include "siface.h"
#include "sim800l.h"
#include "spi.h"
#include "task.h"
#include "timers.h"
#include "usart.h"
#include "w25q_s.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART_BUFFER_SIZE                                \
    (SIM800L_UART_BUFFER_SIZE > SIFACE_UART_BUFFER_SIZE \
         ? SIM800L_UART_BUFFER_SIZE                     \
         : SIFACE_UART_BUFFER_SIZE)

#define STACK_COLOR_WORD 0xACACACAC

/* RTC wakeup clock = LSE / 16 = 2048 Hz (set in MX_RTC_Init) */
#define RTC_WKUP_CLK_HZ 2048U

/* Maximum sleep time limited by IWDG (32 s) with safety margin */
#define MAX_SLEEP_MS 30000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

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

extern const uint32_t* _ebss;
extern const uint32_t* _estack;
extern const uint32_t* _app;
#define APP_ADDRESS ((uint32_t)&_app)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void usb_cdc_rx_callback(uint8_t* buf, size_t size) {
    siface_rx_irq(&siface, (const char*)buf, size);
}

void usb_cdc_tx_callback(void) {
    siface_tx_irq(&siface);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) {
        return;
    }

    //	if (huart == &huart1)
    //	{
    //		siface_rx_irq(&siface, (const char *) ub_sif, size);
    //		HAL_UARTEx_ReceiveToIdle_DMA(huart, ub_sif, UART_BUFFER_SIZE);
    //	}
    if (huart == &huart2) {
        sim800l_irq(&mod, (const char*)ub_mod, size);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, ub_mod, UART_BUFFER_SIZE);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    //	if (huart == &huart1)
    //	{
    //		HAL_UARTEx_ReceiveToIdle_DMA(huart, ub_sif, UART_BUFFER_SIZE);
    //	}
    if (huart == &huart2) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, ub_mod, UART_BUFFER_SIZE);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    //	if (huart == &huart1)
    //	{
    //		siface_tx_irq(&siface);
    //	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (!init_done) {
        return;
    }

    if (GPIO_Pin == EXTI0_HALL_Pin) {
        counter_irq(&cnt);
    }
}

void btn_callback(void) {
    if (!init_done) {
        return;
    }

    task_sensors_notify(&sens);
}

static void sim800l_power_on_cb(void) {
    /* Hold RST low during power-up */
    HAL_GPIO_WritePin(MDM_RST_GPIO_Port, MDM_RST_Pin, GPIO_PIN_RESET);

    /* 2-stage power enable: pre-charge cap through 120 Ohm, then full */
    HAL_GPIO_WritePin(MDM_EN_PRE_GPIO_Port, MDM_EN_PRE_Pin, GPIO_PIN_SET);
    osDelay(200); /* Charge 100uF through 120 Ohm (~60ms for 5*tau) */
    HAL_GPIO_WritePin(MDM_EN_GPIO_Port, MDM_EN_Pin, GPIO_PIN_SET);
    osDelay(100);

    /* Reinitialize UART + start DMA RX (pins back to AF mode) */
    MX_USART2_UART_Init();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, ub_mod, UART_BUFFER_SIZE);

    /* Release RST — module starts booting */
    HAL_GPIO_WritePin(MDM_RST_GPIO_Port, MDM_RST_Pin, GPIO_PIN_SET);
    osDelay(3000); /* Wait for SIM800L boot */
}

static void sim800l_power_off_cb(void) {
    /* Hold RST low before cutting power */
    HAL_GPIO_WritePin(MDM_RST_GPIO_Port, MDM_RST_Pin, GPIO_PIN_RESET);

    /* Deinit UART to release pins (no parasitic power via TX/RX) */
    HAL_UART_DeInit(&huart2);

    HAL_GPIO_WritePin(MDM_EN_GPIO_Port, MDM_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MDM_EN_PRE_GPIO_Port, MDM_EN_PRE_Pin, GPIO_PIN_RESET);
}

static void as5600_power_on_cb(void) {
    HAL_GPIO_WritePin(SENS_EN_GPIO_Port, SENS_EN_Pin, GPIO_PIN_SET);
    osDelay(20);
    MX_I2C1_Init();
}

static void as5600_power_off_cb(void) {
    HAL_I2C_DeInit(&hi2c1);
    HAL_GPIO_WritePin(SENS_EN_GPIO_Port, SENS_EN_Pin, GPIO_PIN_RESET);
}

static void aht20_power_on_cb(void) {
    HAL_GPIO_WritePin(AHT20_EN_GPIO_Port, AHT20_EN_Pin, GPIO_PIN_SET);
    osDelay(100);
    MX_I2C2_Init();
}

static void aht20_power_off_cb(void) {
    HAL_I2C_DeInit(&hi2c2);
    HAL_GPIO_WritePin(AHT20_EN_GPIO_Port, AHT20_EN_Pin, GPIO_PIN_RESET);
}

static void counter_power_on_cb(void) {
    HAL_GPIO_WritePin(HALL_EN_GPIO_Port, HALL_EN_Pin, GPIO_PIN_SET);

    /* Restore EXTI interrupt on Hall sensor pin */
    GPIO_InitTypeDef gi = {0};
    gi.Pin              = EXTI0_HALL_Pin;
    gi.Mode             = GPIO_MODE_IT_FALLING;
    gi.Pull             = GPIO_NOPULL;
    HAL_GPIO_Init(EXTI0_HALL_GPIO_Port, &gi);

    __HAL_GPIO_EXTI_CLEAR_IT(EXTI0_HALL_Pin);
    HAL_NVIC_ClearPendingIRQ(EXTI0_HALL_EXTI_IRQn);
    HAL_NVIC_EnableIRQ(EXTI0_HALL_EXTI_IRQn);
}

static void counter_power_off_cb(void) {
    /* Disable EXTI to prevent spurious wakeups from Stop mode */
    HAL_NVIC_DisableIRQ(EXTI0_HALL_EXTI_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(EXTI0_HALL_Pin);

    /* Reconfigure pin to analog — lowest leakage in Stop mode */
    GPIO_InitTypeDef gi = {0};
    gi.Pin              = EXTI0_HALL_Pin;
    gi.Mode             = GPIO_MODE_ANALOG;
    gi.Pull             = GPIO_NOPULL;
    HAL_GPIO_Init(EXTI0_HALL_GPIO_Port, &gi);

    HAL_GPIO_WritePin(HALL_EN_GPIO_Port, HALL_EN_Pin, GPIO_PIN_RESET);
}

static void avoltage_power_on_cb(void) {
    HAL_GPIO_WritePin(VBAT_MEAS_EN_GPIO_Port, VBAT_MEAS_EN_Pin, GPIO_PIN_SET);
}

static void avoltage_power_off_cb(void) {
    HAL_GPIO_WritePin(VBAT_MEAS_EN_GPIO_Port, VBAT_MEAS_EN_Pin, GPIO_PIN_RESET);
}

static void w25q_hw_init_cb(void) {
    MX_SPI2_Init();
}

static void w25q_hw_deinit_cb(void) {
    HAL_SPI_DeInit(&hspi2);
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
void gpio_enter_stop(void) {
    /* --- W25Q flash: deep power-down BEFORE disabling SPI2 ---
	 * Reduces flash standby from ~25 µA to ~1 µA.
	 * SPI2 must still be functional at this point. */
    w25q_power_down(&mem.mem);

    /* --- Disable DMA1 clock (no active transfers during Stop) --- */
    __HAL_RCC_DMA1_CLK_DISABLE();

    /* --- Disable GPIOH clock (only HSE pins, unused in Stop) --- */
    __HAL_RCC_GPIOH_CLK_DISABLE();
}

/*
 * After waking from Stop, restore peripheral pins and clocks.
 * SystemClock_Config() has already been called at this point.
 */
void gpio_exit_stop(void) {
    /* Re-enable port clocks that were disabled */
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* --- W25Q flash: release deep power-down AFTER SPI2 is restored --- */
    w25q_power_on(&mem.mem);
}

//
static void stack_color(void) {
    volatile uint32_t *p, *end;

    p   = (uint32_t*)&_ebss;
    end = (uint32_t*)__get_MSP();
    p++;

    while (p < end) {
        *p = STACK_COLOR_WORD;
        p++;
    }
}

//
size_t stack_size(void) {
    volatile uint32_t *p, *end;

    p   = (uint32_t*)&_ebss;
    end = (uint32_t*)&_estack;
    p++;

    while ((p < end) && (*p == STACK_COLOR_WORD)) {
        p++;
    }

    return (size_t)end - (size_t)p;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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

    /* Hall sensor starts powered off — disable EXTI0 and set pin to
     * analog so it cannot cause spurious wakeups from Stop mode.
     * counter_power_on_cb() will re-enable it when needed. */
    HAL_NVIC_DisableIRQ(EXTI0_HALL_EXTI_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(EXTI0_HALL_Pin);
    {
        GPIO_InitTypeDef gi = {0};
        gi.Pin              = EXTI0_HALL_Pin;
        gi.Mode             = GPIO_MODE_ANALOG;
        gi.Pull             = GPIO_NOPULL;
        HAL_GPIO_Init(EXTI0_HALL_GPIO_Port, &gi);
    }

    MX_DMA_Init();
    MX_IWDG_Init();
    MX_RTC_Init();
    /* USER CODE BEGIN 2 */

    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

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
    appif.params    = &params;
    appif.actual    = &actual;
    appif.bl        = &bl;
    memcpy(&appif.uparams, &params, sizeof(params));

    //
    button_init(&btn, BTN_MB_GPIO_Port, BTN_MB_Pin, btn_callback);
    siface_init(&siface, 32, appiface, &appif);
#ifdef LOGGER
    logger_init(&logger, &siface);
#endif
    w25q_s_init(&mem,
                &hspi2,
                SPI2_CS_GPIO_Port,
                SPI2_CS_Pin,
                w25q_hw_init_cb,
                w25q_hw_deinit_cb);
    sim800l_init(&mod,
                 &huart2,
                 sim800l_power_on_cb,
                 sim800l_power_off_cb,
                 params.apn);
    ota_init(&ota, &mod, &mem, params.secret, params.url_ota);
    as5600_init(&pot, &hi2c1, as5600_power_on_cb, as5600_power_off_cb);
    aht20_init(&aht, &hi2c2, aht20_power_on_cb, aht20_power_off_cb);
    counter_init(&cnt, counter_power_on_cb, counter_power_off_cb);
    avoltage_init(&avlt,
                  &hadc1,
                  2,
                  avoltage_power_on_cb,
                  avoltage_power_off_cb);

    //
    mqueue_init(&mem);

    // sens
    memset(&sens, 0, sizeof(sens));
    sens.queue =
        mqueue_create(SENSORS_QUEUE_SECNUM, sizeof(struct sensor_record));
    sens.avlt      = &avlt;
    sens.pot       = &pot;
    sens.aht       = &aht;
    sens.cnt       = &cnt;
    sens.timestamp = &timestamp;
    sens.params    = &params;
    sens.actual    = &actual;
    sens.events    = 0;

    // ecnt
    memset(&ecnt, 0, sizeof(ecnt));
    ecnt.cnt    = &cnt;
    ecnt.params = &params;
    ecnt.actual = &actual;

    // app
    memset(&app, 0, sizeof(app));
    app.timestamp = &timestamp;
    app.params    = &params;
    app.sens      = &sens;
    app.mod       = &mod;
    app.bl        = &bl;

    // sys
    memset(&sys, 0, sizeof(sys));
    sys.params = &params;
    sys.bl     = &bl;

    // Flash power-down in Stop mode
    HAL_PWREx_EnableFlashPowerDown();

    //
    /* todo: replace with USB */
    //HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ub_sif, UART_BUFFER_SIZE);
    /* UART2 DMA RX is now started inside sim800l_power_on via hw_init callback */

    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();

    /* Call init function for freertos objects (in cmsis_os2.c) */
    MX_FREERTOS_Init();

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
  */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI |
                                       RCC_OSCILLATORTYPE_HSE |
                                       RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState       = RCC_LSE_ON;
    RCC_OscInitStruct.LSIState       = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 25;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
  */
    RCC_ClkInitStruct.ClockType    = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
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
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {}
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
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
