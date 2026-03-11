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

#include "atomic.h"
#include "iwdg.h"
#include "ptasks.h"
#include "rtc.h"
#include "rtctime.h"

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

extern volatile uint32_t timestamp;
extern params_t params;
extern struct button btn;
extern struct siface siface;
extern struct sensors sens;
extern struct ecounter ecnt;
extern struct sim800l mod;
extern struct ota ota;
extern struct app app;
extern struct system sys;
extern volatile int init_done;

/* USER CODE END Variables */
/* Definitions for def */
osThreadId_t defHandle;
const osThreadAttr_t def_attributes = {
    .name       = "def",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

extern size_t stack_size(void);

/* USER CODE END FunctionPrototypes */

void task_default(void* argument);

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
    gpio_enter_stop();

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
    gpio_exit_stop();

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
    uint32_t sync_cycles = 0;
    TickType_t wake      = xTaskGetTickCount();
    EventBits_t bits;

    /* Infinite loop — 30 s period */
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(30000));

        /* Update global timestamp from RTC (keeps running in Stop mode) */
        timestamp = get_timestamp();
        sync_cycles++;

        /* Refresh IWDG (32 s timeout, 30 s period → safe) */
        IWDG_reset();

        bits = 0;

        /* mtime_count is in seconds; convert 30-s cycles to seconds */
        uint32_t sync_sec = sync_cycles * 30U;

        if (params.mtime_count && (sync_sec % params.mtime_count == 0)) {
            bits |= SYNC_BIT_ECOUNTER;
        }

        if (params.period_sen && sync_sec >= 30 &&
            ((sync_sec - 30) % params.period_sen == 0)) {
            bits |= SYNC_BIT_SENSORS;
        }

        /* APP fires one cycle after the aligned boundary so that sensors
         * have time to read and write fresh data before app reads the queue */
        if (params.period_app && sync_sec >= 60 &&
            ((sync_sec - 60) % params.period_app == 0)) {
            bits |= SYNC_BIT_APP;
        }

        if (bits) {
            xEventGroupSetBits(sync_events, bits);
        }
    }
    /* USER CODE END task_default */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
