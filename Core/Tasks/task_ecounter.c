/*
 * Extended counter task
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2025-2026
 */

#include "ptasks.h"

/* Measurement window duration (ms) */
#define COUNTER_MEAS_TIME_MS 3000

/* Stabilization time after power on (ms) */
#define COUNTER_STABILIZE_MS 10

/* Polling interval while waiting for pulses (ms) */
#define COUNTER_POLL_MS 50

static osThreadId_t handle;
static const osThreadAttr_t attributes = {
  .name = "ecounter",
  .stack_size = 112 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};


static void task(void *argument)
{
	struct ecounter *ecnt = argument;
	uint32_t value;
	uint32_t elapsed;

	for (;;)
	{
		/* Wait for sync signal from hz_timer */
		xEventGroupWaitBits(sync_events, SYNC_BIT_ECOUNTER,
				pdTRUE, pdFALSE, portMAX_DELAY);

		led_blink(2);

		/* Power on and stabilize */
		counter_power_on(ecnt->cnt);
		osDelay(COUNTER_STABILIZE_MS);

		/* Reset counter before measurement */
		counter_reset(ecnt->cnt);

		/*
		 * Wait for enough pulses or timeout.
		 * - Exit early once we have COUNTER_MIN_PERIODS periods
		 * - If no first pulse by half the window, abort early
		 * - Full timeout as fallback
		 */
		elapsed = 0;
		while (elapsed < COUNTER_MEAS_TIME_MS) {
			osDelay(COUNTER_POLL_MS);
			elapsed += COUNTER_POLL_MS;

			/* Enough periods collected - done */
			if (ecnt->cnt->period_cnt >= COUNTER_MIN_PERIODS)
				break;

			/* Half window passed with no pulse - no point waiting */
			if (elapsed >= COUNTER_MEAS_TIME_MS / 2 &&
			    !ecnt->cnt->started)
				break;
		}

		/* Convert to speed: higher value = faster rotation */
		value = counter_speed(ecnt->cnt);

		/* Power off - sleep between measurements */
		counter_power_off(ecnt->cnt);

		/* Update actual value */
		xSemaphoreTake(ecnt->actual->mutex, portMAX_DELAY);
		ecnt->actual->count = value;
		xSemaphoreGive(ecnt->actual->mutex);

		/* Update accumulator for min/avg/max aggregation */
		counter_accum_update(ecnt->cnt, value);
	}
}

/******************************************************************************/
void task_ecounter(struct ecounter *ecnt)
{
	handle = osThreadNew(task, ecnt, &attributes);
}

