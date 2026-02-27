/*
 * SIM800L library
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#ifndef SIM800L_H_
#define SIM800L_H_

#include <stdbool.h>

#include "stm32f4xx_hal.h"

#include "cmsis_os.h"
#include "queue.h"
#include "stream_buffer.h"

#define SIM800L_BUFFER_SIZE 1024
#define SIM800L_UART_BUFFER_SIZE 128

#define SIM800L_TASK_QUEUE_SIZE 10

#define SIM800L_APN_SIZE 32

#define SIM800L_NETSCAN_DONE 1

typedef uint64_t timeout_t;

typedef void (*sim800l_cb)(int, void *);
typedef void (*sim800l_hw_init)(void);

/*
 * @brief: SIM800L task structure
 * TODO: fields description
 */
struct sim800l_task
{
	int issue;
	TickType_t timeout;
	sim800l_cb callback;
	void *data;
};

/*
 * @brief: Base SIM800L structure
 * TODO: fields description
 */
struct sim800l
{
	UART_HandleTypeDef *uart;
	sim800l_hw_init hw_init;
	GPIO_TypeDef *rst_port;
	uint16_t rst_pin;

	GPIO_TypeDef *pwr_port;
	uint16_t pwr_pin;
	GPIO_TypeDef *pwr_pre_port;
	uint16_t pwr_pre_pin;

	StreamBufferHandle_t stream;
	xQueueHandle queue;

	int state;
	int errors;

	uint8_t txb[SIM800L_BUFFER_SIZE + 1]; // + 1 for additional '\r'
	uint8_t rxb[SIM800L_BUFFER_SIZE + 1]; // + 1 for additional '\0'

	size_t rxlen;

	TickType_t task_ticks;
	struct sim800l_task task;

	char apn[SIM800L_APN_SIZE];

	int bcl;
	int voltage;
};

/*
 * @brief: Voltage measurement request structure
 * TODO: fields description
 */
struct sim800l_voltage
{
	int voltage;

	void *context;
};

/*
 * @brief: HTTP-request structure
 * TODO: fields description
 */
struct sim800l_http
{
	char *url;
	char *request;
	char *response;
	size_t rlen;

	char *req_auth;
	char *res_auth;
	bool res_auth_get;

	void *context;
};

/*
 * @brief: Net scan request structure
 * TODO: fields description
 */
struct sim800l_netscan
{
	int32_t mcc;
	int32_t mnc;
	int32_t lac;
	int32_t cid;
	int32_t lev;

	void *context;
};


/*
 * @brief: struct sim800l handle initialization
 * @param mod: struct sim800l handle
 * @param uart: UART_HandleTypeDef handle that is used for interaction
 * @param hw_init: UART hardware init callback (reinitializes UART + starts DMA RX)
 * @param rst_port: GPIO RESET port
 * @param rst_pin: GPIO RESET pin
 * @param pwr_port: GPIO power enable port (MDM_EN)
 * @param pwr_pin: GPIO power enable pin (MDM_EN)
 * @param pwr_pre_port: GPIO pre-charge power enable port (MDM_EN_PRE)
 * @param pwr_pre_pin: GPIO pre-charge power enable pin (MDM_EN_PRE)
 * @param apn: APN string
 */
void sim800l_init(struct sim800l *mod, UART_HandleTypeDef *uart,
		sim800l_hw_init hw_init,
		GPIO_TypeDef *rst_port, uint16_t rst_pin,
		GPIO_TypeDef *pwr_port, uint16_t pwr_pin,
		GPIO_TypeDef *pwr_pre_port, uint16_t pwr_pre_pin,
		char *apn);

/*
 * @brief: Copy data from UART interrupt handler
 * @param mod: struct sim800l handle
 * @param buf: data buffer
 * @param len: data buffer length (in bytes)
 */
void sim800l_irq(struct sim800l *mod, const char *buf, size_t len);

/*
 * @brief: SIM800L task
 * @param mod: struct sim800l handle
 */
void sim800l_task(struct sim800l *mod);

/*
 * @brief: Power on SIM800L (2-stage: EN_PRE then EN)
 * @param mod: struct sim800l handle
 */
void sim800l_power_on(struct sim800l *mod);

/*
 * @brief: Power off SIM800L
 * @param mod: struct sim800l handle
 */
void sim800l_power_off(struct sim800l *mod);

/*
 * @brief: Lock SIM800L in sleep state
 * @param mod: struct sim800l handle
 * @retval: 0 on success, -1 on failure (SIM800L is busy and can not sleep)
 */
int sim800l_sleep_lock(struct sim800l *mod);

/*
 * @brief: Unlock SIM800L in sleep state
 * @param mod: struct sim800l handle
 */
void sim800l_sleep_unlock(struct sim800l *mod);

/*
 * @brief: Add voltage measurement request to SIM800L task queue
 * @param data: Request parameters
 * @param callback: User callback after successful measurement or timeout
 * @param timeout: Timeout in ms
 * @retval: 0 on success, -1 on failure
 */
int sim800l_voltage(struct sim800l *mod, struct sim800l_voltage *data,
		sim800l_cb callback, timeout_t timeout);

/*
 * @brief: Add HTTP request to SIM800L task queue
 * @param mod: struct sim800l handle
 * @param data: Request parameters
 * @param callback: User callback after receiving an HTTP response or timeout
 * @param timeout: Timeout in ms
 * @retval: 0 on success, -1 on failure
 */
int sim800l_http(struct sim800l *mod, struct sim800l_http *data,
		sim800l_cb callback, timeout_t timeout);

/*
 * @brief: Add net scan request to SIM800L task queue
 * @param mod: struct sim800l handle
 * @param data: Request parameters
 * @param callback: User callback after receiving every set of net parameters,
 *     net scan done or timeout
 * @param timeout: Timeout in ms
 * @retval: 0 on success, -1 on failure
 */
int sim800l_netscan(struct sim800l *mod, struct sim800l_netscan *data,
		sim800l_cb callback, timeout_t timeout);

#endif /* SIM800L_H_ */
