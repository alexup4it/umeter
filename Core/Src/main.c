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

#include "actual.h"
#include "adc.h"
#include "aht20.h"
#include "appiface.h"
#include "as5600.h"
#include "avoltage.h"
#include "button.h"
#include "freqmeter.h"
#include "fws.h"
#include "i2c.h"
#include "logger.h"
#include "params.h"
#include "ptasks.h"
#include "sensorq.h"
#include "siface.h"
#include "sim800l.h"
#include "spi.h"
#include "usart.h"
#include "w25q_s.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

volatile int init_done = 0;

extern const uint32_t* _ebss;
extern const uint32_t* _estack;
extern const uint32_t* _app;
#define APP_ADDRESS ((uint32_t)&_app)

/* Hardware instances */
static struct actual actual;
static struct as5600 pot;
static struct aht20 aht;
static struct freqmeter cnt;
static struct avoltage avlt;
static struct button btn;
static struct siface siface;
static struct sim800l modem;
static struct w25q_s mem;
static struct sensorq sensorq;
static struct sensor_record sensorq_buf[SENSORS_QUEUE_CAPACITY];
#ifdef LOGGER
static struct logger logger;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*---------------------------------------------------------------------------*/
/* Power management functions                                                */
/*---------------------------------------------------------------------------*/

static void pm_modem_on(void) {
    HAL_GPIO_WritePin(MDM_RST_GPIO_Port, MDM_RST_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MDM_EN_PRE_GPIO_Port, MDM_EN_PRE_Pin, GPIO_PIN_SET);
    osDelay(pdMS_TO_TICKS(200));
    HAL_GPIO_WritePin(MDM_EN_GPIO_Port, MDM_EN_Pin, GPIO_PIN_SET);
    osDelay(pdMS_TO_TICKS(100));
    MX_USART2_UART_Init();
    HAL_GPIO_WritePin(MDM_RST_GPIO_Port, MDM_RST_Pin, GPIO_PIN_SET);
    osDelay(pdMS_TO_TICKS(3000));
}

static void pm_modem_off(void) {
    HAL_GPIO_WritePin(MDM_RST_GPIO_Port, MDM_RST_Pin, GPIO_PIN_RESET);
    HAL_UART_DeInit(&huart2);
    HAL_GPIO_WritePin(MDM_EN_GPIO_Port, MDM_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MDM_EN_PRE_GPIO_Port, MDM_EN_PRE_Pin, GPIO_PIN_RESET);
}

static void pm_as5600_on(void) {
    HAL_GPIO_WritePin(SENS_EN_GPIO_Port, SENS_EN_Pin, GPIO_PIN_SET);
    osDelay(pdMS_TO_TICKS(20));
    MX_I2C1_Init();
}

static void pm_as5600_off(void) {
    HAL_I2C_DeInit(&hi2c1);
    HAL_GPIO_WritePin(SENS_EN_GPIO_Port, SENS_EN_Pin, GPIO_PIN_RESET);
}

static void pm_aht20_on(void) {
    MX_I2C2_Init();
    HAL_GPIO_WritePin(AHT20_EN_GPIO_Port, AHT20_EN_Pin, GPIO_PIN_SET);
    osDelay(pdMS_TO_TICKS(100));
}

static void pm_aht20_off(void) {
    HAL_I2C_DeInit(&hi2c2);
    HAL_GPIO_WritePin(AHT20_EN_GPIO_Port, AHT20_EN_Pin, GPIO_PIN_RESET);
}

static void pm_anemometer_on(void) {
    HAL_GPIO_WritePin(HALL_EN_GPIO_Port, HALL_EN_Pin, GPIO_PIN_SET);

    GPIO_InitTypeDef gi = {0};
    gi.Pin              = EXTI0_HALL_Pin;
    gi.Mode             = GPIO_MODE_IT_FALLING;
    gi.Pull             = GPIO_NOPULL;
    HAL_GPIO_Init(EXTI0_HALL_GPIO_Port, &gi);

    __HAL_GPIO_EXTI_CLEAR_IT(EXTI0_HALL_Pin);
    HAL_NVIC_ClearPendingIRQ(EXTI0_HALL_EXTI_IRQn);
    HAL_NVIC_EnableIRQ(EXTI0_HALL_EXTI_IRQn);
}

static void pm_anemometer_off(void) {
    HAL_NVIC_DisableIRQ(EXTI0_HALL_EXTI_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(EXTI0_HALL_Pin);

    GPIO_InitTypeDef gi = {0};
    gi.Pin              = EXTI0_HALL_Pin;
    gi.Mode             = GPIO_MODE_ANALOG;
    gi.Pull             = GPIO_NOPULL;
    HAL_GPIO_Init(EXTI0_HALL_GPIO_Port, &gi);

    HAL_GPIO_WritePin(HALL_EN_GPIO_Port, HALL_EN_Pin, GPIO_PIN_RESET);
}

static void pm_avoltage_on(void) {
    MX_ADC1_Init();
    HAL_GPIO_WritePin(VBAT_MEAS_EN_GPIO_Port, VBAT_MEAS_EN_Pin, GPIO_PIN_SET);
    osDelay(pdMS_TO_TICKS(10));
}

static void pm_avoltage_off(void) {
    HAL_ADC_DeInit(&hadc1);
    HAL_GPIO_WritePin(VBAT_MEAS_EN_GPIO_Port, VBAT_MEAS_EN_Pin, GPIO_PIN_RESET);
}

static void pm_flash_spi_init(void) {
    MX_SPI2_Init();
}

static void pm_flash_spi_deinit(void) {
    HAL_SPI_DeInit(&hspi2);
}

void pm_flash_enter_stop(void) {
    w25q_power_down(&mem.mem);
    __HAL_RCC_DMA1_CLK_DISABLE();
    __HAL_RCC_GPIOH_CLK_DISABLE();
}

void pm_flash_exit_stop(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    w25q_power_on(&mem.mem);
}

/*---------------------------------------------------------------------------*/
/* Task context instances                                                    */
/*---------------------------------------------------------------------------*/

struct task_default_ctx task_default_ctx = {
    .logger  = &logger,
    .sensorq = &sensorq,
};
struct task_blink_ctx task_blink_ctx = {0};

struct task_button_ctx task_button_ctx = {
    .actual = &actual,
    .btn    = &btn,
};

struct task_watchdog_ctx task_watchdog_ctx = {0};

struct task_anemometer_ctx task_anemometer_ctx = {
    .actual         = &actual,
    .cnt            = &cnt,
    .anemometer_on  = pm_anemometer_on,
    .anemometer_off = pm_anemometer_off,
};

struct task_sensors_ctx task_sensors_ctx = {
    .actual  = &actual,
    .queue   = &sensorq,
    .as5600  = &pot,
    .aht20   = &aht,
    .voltage = &avlt,
    .cnt     = &cnt,
#ifdef LOGGER
    .logger = &logger,
#endif
    .as5600_on    = pm_as5600_on,
    .as5600_off   = pm_as5600_off,
    .aht20_on     = pm_aht20_on,
    .aht20_off    = pm_aht20_off,
    .avoltage_on  = pm_avoltage_on,
    .avoltage_off = pm_avoltage_off,
};

struct task_modem_ctx task_modem_ctx = {
    .actual    = &actual,
    .modem     = &modem,
    .power_on  = pm_modem_on,
    .power_off = pm_modem_off,
#ifdef LOGGER
    .logger = &logger,
#endif
};

struct task_serial_iface_ctx task_serial_iface_ctx = {
    .siface = &siface,
};

struct task_logging_ctx task_logging_ctx = {
#ifdef LOGGER
    .logger = &logger,
#endif
};

struct task_net_ctx task_net_ctx = {
    .actual = &actual,
    .queue  = &sensorq,
#ifdef LOGGER
    .logger = &logger,
#endif
};

struct task_ota_ctx task_ota_ctx = {
    .mem = &mem,
#ifdef LOGGER
    .logger = &logger,
#endif
};

/*---------------------------------------------------------------------------*/
/* ISR forwarders                                                            */
/*---------------------------------------------------------------------------*/

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

    if (huart == &huart2) {
        sim800l_irq(&modem, size);
        HAL_UARTEx_ReceiveToIdle_DMA(modem.uart,
                                     (uint8_t*)modem.dma_buffer,
                                     SIM800L_UART_BUFFER_SIZE);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (huart == &huart2) {
        HAL_UART_AbortReceive(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(modem.uart,
                                     (uint8_t*)modem.dma_buffer,
                                     SIM800L_UART_BUFFER_SIZE);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    (void)huart;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    GPIO_PinState pin_state;

    if (!init_done) {
        return;
    }

    if (GPIO_Pin == EXTI0_HALL_Pin) {
        freqmeter_irq(&cnt);
        return;
    }

    if (GPIO_Pin == BTN_Pin) {
        pin_state = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
        if (button_irq_callback(&btn,
                                (pin_state == GPIO_PIN_RESET),
                                HAL_GetTick())) {
            task_button_irq_notify_from_isr();
        }
    }
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
    MX_DMA_Init();
    MX_IWDG_Init();
    MX_RTC_Init();
    /* USER CODE BEGIN 2 */

    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    //
    params_init();

    //
    actual_init(&actual);

    //
    appif.actual  = &actual;
    appif.uparams = params;

    //
    button_init(&btn, NULL, 50);
    siface_init(&siface, 32, appiface, &appif);
#ifdef LOGGER
    logger_init(&logger, &siface);
#endif
    w25q_s_init(&mem,
                &hspi2,
                SPI2_CS_GPIO_Port,
                SPI2_CS_Pin,
                pm_flash_spi_init,
                pm_flash_spi_deinit);
    sim800l_init(&modem, &huart2, params.apn, &logger);
    modem_init();
    as5600_init(&pot, &hi2c1);
    aht20_init(&aht, &hi2c2);
    freqmeter_init(&cnt);
    avoltage_init(&avlt, &hadc1, 2);

    sensorq_init(&sensorq, sensorq_buf, SENSORS_QUEUE_CAPACITY);

    // Flash power-down in Stop mode
    HAL_PWREx_EnableFlashPowerDown();

    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize(); /* Call init function for freertos objects (in cmsis_os2.c) */
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
