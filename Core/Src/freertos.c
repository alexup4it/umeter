/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"

#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "iwdg.h"
#include "ptasks.h"
#include "rtc.h"

extern void pm_flash_enter_stop(void);
extern void pm_flash_exit_stop(void);

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
/* USER CODE BEGIN Variables */

extern volatile int init_done;

/* Context instances (defined in main.c) */
extern struct task_default_ctx task_default_ctx;
extern struct task_blink_ctx task_blink_ctx;
extern struct task_button_ctx task_button_ctx;
extern struct task_watchdog_ctx task_watchdog_ctx;
extern struct task_anemometer_ctx task_anemometer_ctx;
extern struct task_sensors_ctx task_sensors_ctx;
extern struct task_modem_ctx task_modem_ctx;
extern struct task_serial_iface_ctx task_serial_iface_ctx;
extern struct task_logging_ctx task_logging_ctx;
extern struct task_net_ctx task_net_ctx;
extern struct task_ota_ctx task_ota_ctx;

/* USER CODE END Variables */
/* Definitions for def */
osThreadId_t defHandle;
const osThreadAttr_t def_attributes = {
    .name       = "def",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityLow,
};
/* Definitions for blink */
osThreadId_t blinkHandle;
const osThreadAttr_t blink_attributes = {
    .name       = "blink",
    .stack_size = 64 * 4,
    .priority   = (osPriority_t)osPriorityLow,
};
/* Definitions for button */
osThreadId_t buttonHandle;
const osThreadAttr_t button_attributes = {
    .name       = "button",
    .stack_size = 64 * 4,
    .priority   = (osPriority_t)osPriorityBelowNormal,
};
/* Definitions for logging */
osThreadId_t loggingHandle;
const osThreadAttr_t logging_attributes = {
    .name       = "logging",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityBelowNormal,
};
/* Definitions for modem */
osThreadId_t modemHandle;
const osThreadAttr_t modem_attributes = {
    .name       = "modem",
    .stack_size = 256 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for serial_iface */
osThreadId_t serial_ifaceHandle;
const osThreadAttr_t serial_iface_attributes = {
    .name       = "serial_iface",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for ota */
osThreadId_t otaHandle;
const osThreadAttr_t ota_attributes = {
    .name       = "ota",
    .stack_size = 512 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for anemometer */
osThreadId_t anemometerHandle;
const osThreadAttr_t anemometer_attributes = {
    .name       = "anemometer",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for sensors */
osThreadId_t sensorsHandle;
const osThreadAttr_t sensors_attributes = {
    .name       = "sensors",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for watchdog */
osThreadId_t watchdogHandle;
const osThreadAttr_t watchdog_attributes = {
    .name       = "watchdog",
    .stack_size = 64 * 4,
    .priority   = (osPriority_t)osPriorityHigh,
};
/* Definitions for net */
osThreadId_t netHandle;
const osThreadAttr_t net_attributes = {
    .name       = "net",
    .stack_size = 512 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void task_default(void* argument);
extern void task_blink(void* argument);
extern void task_button(void* argument);
extern void task_logging(void* argument);
extern void task_modem(void* argument);
extern void task_serial_iface(void* argument);
extern void task_ota(void* argument);
extern void task_anemometer(void* argument);
extern void task_sensors(void* argument);
extern void task_watchdog(void* argument);
extern void task_net(void* argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN VPORT_SUPPORT_TICKS_AND_SLEEP */

/*
 * Custom tickless idle: RTC wakeup timer + Stop mode
 *
 * Overrides the CubeMX-generated weak stub to allow sleep durations of up to
 * MAX_SLEEP_TICKS (30 s), far beyond the SysTick 24-bit counter limit (~200 ms
 * at 84 MHz).
 *
 * RTC wakeup timer clocked from LSE/16 = 2048 Hz (16-bit counter) gives a
 * maximum single-shot period of 65535/2048 ≈ 32 s.
 *
 * Elapsed time is measured via RTC calendar registers (sub-second resolution
 * ≈ 4 ms with SynchPrediv=255) which keeps ticking during Stop mode, so the
 * RTOS tick count stays accurate after waking.
 */

/* RTC wakeup clock = LSE / 16 = 2048 Hz */
#define WKUP_CLK_HZ 2048U
/* Max sleep capped below IWDG timeout (32 s) */
#define MAX_SLEEP_TICKS pdMS_TO_TICKS(30000U)

/* Convert RTC time to milliseconds-of-day (0..86 399 999) */
static uint32_t rtc_time_to_ms(const RTC_TimeTypeDef* t) {
    uint32_t ms = ((uint32_t)t->Hours * 3600U + (uint32_t)t->Minutes * 60U +
                   (uint32_t)t->Seconds) *
                  1000U;
    /* SubSeconds counts down from SynchPrediv (255) to 0 */
    ms += ((uint32_t)(255U - t->SubSeconds) * 1000U) / 256U;
    return ms;
}

void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime) {
    /* Cap to MAX_SLEEP_TICKS for IWDG safety */
    if (xExpectedIdleTime > MAX_SLEEP_TICKS) {
        xExpectedIdleTime = MAX_SLEEP_TICKS;
    }

    /* Disable interrupts (same pattern as port.c) */
    __disable_irq();
    __DSB();
    __ISB();

    if (eTaskConfirmSleepModeStatus() == eAbortSleep) {
        __enable_irq();
        return;
    }

    /*
     * Stop mode is unsafe when:
     *  - USB OTG FS is active (clocks halt -> host sees disconnect)
     *  - SIM800L modem is on  (UART DMA loses sync after HSI wakeup)
     * Fall back to regular WFI — SysTick keeps running, so just return
     * and let FreeRTOS handle the single-tick accounting.
     */
    if (__HAL_RCC_USB_OTG_FS_IS_CLK_ENABLED() ||
        HAL_GPIO_ReadPin(MDM_EN_GPIO_Port, MDM_EN_Pin) == GPIO_PIN_SET) {
        __enable_irq();
        __WFI();
        return;
    }

    /* Refresh IWDG before entering long sleep */
    IWDG_reset();

    /* --- Record RTC time before sleep --- */
    RTC_TimeTypeDef t_before;
    RTC_DateTypeDef d_before;
    HAL_RTC_GetTime(&hrtc, &t_before, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &d_before, RTC_FORMAT_BIN);
    uint32_t ms_before = rtc_time_to_ms(&t_before);

    /* Suspend HAL tick (TIM1) */
    HAL_SuspendTick();

    /* Put peripherals into low-leak configuration */
    pm_flash_enter_stop();

    /* Calculate wakeup timer value and arm it with interrupt */
    {
        uint32_t sleep_ms =
            (uint32_t)xExpectedIdleTime * (1000U / configTICK_RATE_HZ);
        uint32_t wut = (sleep_ms * WKUP_CLK_HZ) / 1000U;
        if (wut > 0) {
            wut--;
        }
        if (wut > 0xFFFF) {
            wut = 0xFFFF;
        }
        HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, wut, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
    }

    /* --- Enter Stop mode (low-power regulator, WFI entry) --- */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    /* === Woken up (interrupts still globally disabled) === */

    /* Deactivate wakeup timer (clears WUTF, disables IT & EXTI) */
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    /* Restore system clock (HSE + PLL) — required after Stop mode */
    SystemClock_Config();

    /* Restore peripheral & GPIO configuration */
    pm_flash_exit_stop();

    /* Resume HAL tick */
    HAL_ResumeTick();

    /* --- Compute elapsed time from RTC --- */
    RTC_TimeTypeDef t_after;
    RTC_DateTypeDef d_after;
    HAL_RTC_GetTime(&hrtc, &t_after, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &d_after, RTC_FORMAT_BIN);
    uint32_t ms_after = rtc_time_to_ms(&t_after);

    /* Handle midnight rollover */
    if (ms_after < ms_before) {
        ms_after += 24 * 60 * 60 * 1000;
    }
    uint32_t elapsed_ms = ms_after - ms_before;

    /* Convert to ticks: elapsed_ms * configTICK_RATE_HZ / 1000 */
    TickType_t elapsed_ticks =
        (TickType_t)(elapsed_ms * configTICK_RATE_HZ / 1000U);

    /* Clamp: vTaskStepTick asserts elapsed < xExpectedIdleTime */
    if (elapsed_ticks >= xExpectedIdleTime) {
        elapsed_ticks = xExpectedIdleTime - 1U;
    }

    /* Re-enable interrupts (pending RTC_WKUP IRQ will fire now) */
    __enable_irq();

    /* Advance RTOS tick count by the time we slept */
    if (elapsed_ticks > 0) {
        vTaskStepTick(elapsed_ticks);
    }
}

/* USER CODE END VPORT_SUPPORT_TICKS_AND_SLEEP */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */

    task_manager_init();

    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of def */
    defHandle =
        osThreadNew(task_default, (void*)&task_default_ctx, &def_attributes);

    /* creation of blink */
    blinkHandle =
        osThreadNew(task_blink, (void*)&task_blink_ctx, &blink_attributes);

    /* creation of button */
    buttonHandle =
        osThreadNew(task_button, (void*)&task_button_ctx, &button_attributes);

    /* creation of logging */
    loggingHandle = osThreadNew(task_logging,
                                (void*)&task_logging_ctx,
                                &logging_attributes);

    /* creation of modem */
    modemHandle =
        osThreadNew(task_modem, (void*)&task_modem_ctx, &modem_attributes);

    /* creation of serial_iface */
    serial_ifaceHandle = osThreadNew(task_serial_iface,
                                     (void*)&task_serial_iface_ctx,
                                     &serial_iface_attributes);

    /* creation of ota */
    otaHandle = osThreadNew(task_ota, (void*)&task_ota_ctx, &ota_attributes);

    /* creation of anemometer */
    anemometerHandle = osThreadNew(task_anemometer,
                                   (void*)&task_anemometer_ctx,
                                   &anemometer_attributes);

    /* creation of sensors */
    sensorsHandle = osThreadNew(task_sensors,
                                (void*)&task_sensors_ctx,
                                &sensors_attributes);

    /* creation of watchdog */
    watchdogHandle = osThreadNew(task_watchdog,
                                 (void*)&task_watchdog_ctx,
                                 &watchdog_attributes);

    /* creation of net */
    netHandle = osThreadNew(task_net, (void*)&task_net_ctx, &net_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */

    init_done = 1;

    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_task_default */
/**
  * @brief  Function implementing the def thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_task_default */
void task_default(void* argument) {
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN task_default */

    task_manager_run();

    /* USER CODE END task_default */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
