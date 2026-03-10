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
#include "ptasks.h"

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
    uint32_t sync_sec = 0;
    TickType_t wake   = xTaskGetTickCount();
    EventBits_t bits;

    /* Infinite loop */
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000));
        atomic_inc(&timestamp);
        sync_sec++;
        bits = 0;

        if (params.mtime_count && (sync_sec % params.mtime_count == 0)) {
            bits |= SYNC_BIT_ECOUNTER;
        }

        if (params.period_sen && sync_sec >= 5 &&
            ((sync_sec - 5) % params.period_sen == 0)) {
            bits |= SYNC_BIT_SENSORS;
        }

        /* APP fires 10 seconds after the aligned boundary so that sensors
     * have time to read and write fresh data before app reads the queue */
        if (params.period_app && sync_sec >= 10 &&
            ((sync_sec - 10) % params.period_app == 0)) {
            bits |= SYNC_BIT_APP;
        }

        if (sync_sec % 5 == 0) {
            led_blink(1);
        }

        bits |= SYNC_BIT_WATCHDOG;

        xEventGroupSetBits(sync_events, bits);
    }
    /* USER CODE END task_default */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
