/*
 * SIM800L library
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#include "sim800l.h"

#include <stdlib.h>
#include <string.h>

#include "task.h"

#define MAX_ERRORS 5

#define CMD_BUFFER_SIZE 96

/*
 * Minimum functionality mode
 * Without this feature power consumption will be about 1 mA, but module will be
 * always connected to cellular network. Use only if SIM800L is used rarely
 */
#define DISABLE_RF

#include "logger.h"
#ifdef LOGGER
#define TAG "SIM800L"
extern struct logger logger;
#endif

enum status
{
	STATUS_OK,
	STATUS_ERROR,
};

enum state
{
	STATE_STARTUP,
	STATE_RESET,
	STATE_AT,
	STATE_FLUSH,
	STATE_ECHO_OFF,
	STATE_DEL_SMS,
	STATE_IDLE,
	STATE_DO_TASK,
	STATE_CBC,
	STATE_CREG,
	STATE_GPRS_INIT,
	STATE_GPRS_HTTP,
	STATE_GPRS_HTTP_TERM,
	STATE_GPRS_DEINIT,
	STATE_NETSCAN,
};

enum issue
{
	ISSUE_IDLE,
	ISSUE_VOLTAGE,
	ISSUE_HTTP,
	ISSUE_NETSCAN,
};


inline static void task_done(struct sim800l *mod)
{
	mod->task.issue = ISSUE_IDLE;
}

inline static void reset_set(struct sim800l *mod)
{
	HAL_GPIO_WritePin(mod->rst_port, mod->rst_pin, GPIO_PIN_RESET);
}

inline static void reset_unset(struct sim800l *mod)
{
	HAL_GPIO_WritePin(mod->rst_port, mod->rst_pin, GPIO_PIN_SET);
}

/******************************************************************************/
void sim800l_init(struct sim800l *mod, UART_HandleTypeDef *uart,
		GPIO_TypeDef *rst_port, uint16_t rst_pin, char *apn)
{
	memset(mod, 0, sizeof(*mod));
	mod->uart = uart;
	mod->rst_port = rst_port;
	mod->rst_pin = rst_pin;
	mod->stream = xStreamBufferCreate(SIM800L_BUFFER_SIZE, 1);
	mod->queue = xQueueCreate(SIM800L_TASK_QUEUE_SIZE,
			sizeof(struct sim800l_task));

	strncpy(mod->apn, apn, sizeof(mod->apn) - 1);
	mod->apn[sizeof(mod->apn) - 1] = '\0';

	reset_unset(mod);
	task_done(mod);
}

/******************************************************************************/
void sim800l_irq(struct sim800l *mod, const char *buf, size_t len)
{
	BaseType_t woken = pdFALSE;
	xStreamBufferSendFromISR(mod->stream, buf, len, &woken);
}

inline static void clear_rx_buffer(struct sim800l *mod)
{
	xStreamBufferReset(mod->stream);
	mod->rxlen = 0;
}

static bool wait_for_any(struct sim800l *mod, timeout_t timeout)
{
	size_t received;

	if (mod->rxlen >= SIM800L_BUFFER_SIZE)
		return false;

	received = xStreamBufferReceive(mod->stream, &mod->rxb[mod->rxlen],
			SIM800L_BUFFER_SIZE - mod->rxlen, pdMS_TO_TICKS(timeout));
	mod->rxlen += received;

	if (!received)
		return false;

#ifdef LOGGER
	logger_add(&logger, TAG, false, (char *) mod->rxb, mod->rxlen);
#endif

	return true;
}

/**
 * @note: timeout should be more then 0
 */
static bool wait_for_num(struct sim800l *mod, size_t num, timeout_t timeout)
{
	TickType_t ticks = xTaskGetTickCount();
	TickType_t left = pdMS_TO_TICKS(timeout);
	TickType_t wait;

	if (num > SIM800L_BUFFER_SIZE)
		return false;

	while ((mod->rxlen < num) && left)
	{
		mod->rxlen += xStreamBufferReceive(mod->stream, &mod->rxb[mod->rxlen],
				num - mod->rxlen, left);

		wait = xTaskGetTickCount() - ticks;
		if (left > wait)
			left -= wait;
		else
			left = 0;
	}

#ifdef LOGGER
	logger_add(&logger, TAG, false, (char *) mod->rxb, mod->rxlen);
#endif

	if (mod->rxlen < num)
		return false;

	return true;
}

static bool compare_buffer_beginning(struct sim800l *mod, const char *sample,
		timeout_t timeout)
{
	size_t len = strlen(sample);

	if (!wait_for_num(mod, len, timeout))
		return false;

	if (strncmp((char *) mod->rxb, sample, len) != 0)
		return false;

	return true;
}

static void transmit_data(struct sim800l *mod, const uint8_t *buf, size_t len)
{
	if (len > SIM800L_BUFFER_SIZE)
		len = SIM800L_BUFFER_SIZE;

	memcpy(mod->txb, buf, len);

	mod->txb[len] = '\r';
	len++;

#ifdef LOGGER
	logger_add(&logger, TAG, false, (char *) mod->txb, len);
#endif

	clear_rx_buffer(mod); // ?
	while (HAL_UART_Transmit_DMA(mod->uart, mod->txb, len) == HAL_BUSY);
}

inline static void transmit(struct sim800l *mod, const char *buf)
{
	transmit_data(mod, (const uint8_t *) buf, strlen(buf));
}

static void state(struct sim800l *mod, enum state new_state, enum status status)
{
	mod->state = new_state;

	if (status != STATUS_OK)
	{
		mod->errors++;
		if (mod->errors >= MAX_ERRORS)
		{
			mod->errors = 0;
			mod->state = STATE_STARTUP;
		}
	}

	// Task timeout
	if (mod->task.issue != ISSUE_IDLE)
	{
		if ((xTaskGetTickCount() - mod->task_ticks) > mod->task.timeout)
		{
			mod->task.callback(-1, mod->task.data);
			task_done(mod);

			mod->state = STATE_STARTUP;
		}
	}
}

// "\r\n+CBC: 0,61,3895\r\n\r\nOK\r\n"
static bool parse_battery_charge(struct sim800l *mod, timeout_t timeout)
{
	const char *header = "+CBC:";
	const char *ending = "\r\nOK\r\n";
	char *p, *end;

	while (wait_for_any(mod, timeout))
	{
		mod->rxb[mod->rxlen] = '\0';

		p = strstr((char *) mod->rxb, header);
		if (!p)
			continue;

		end = strstr(p, ending);
		if (!end)
			continue;

		p = strstr(p, ",");
		if (!p)
			return false;

		p++;
		if (!*p)
			return false;

		mod->bcl = strtoul(p, &p, 0);
		if (!*p)
			return false;

		p++;
		if (!*p)
			return false;

		mod->voltage = strtoul(p, NULL, 0);

		return true;
	}

	return false;
}

// "\r\nOK\r\n\r\n+HTTPACTION: 0,200,293\r\n"
// "\r\nOK\r\n 1,200,16\r\n" ?
// @retval: HTTP status code on success or -1 on failure
static int parse_http_action(struct sim800l *mod, int* len, timeout_t timeout)
{
	const char *header = "+HTTPACTION:";
	const char *ending = "\r\n";
	char *p, *end;
	int status;

	while (wait_for_any(mod, timeout))
	{
		mod->rxb[mod->rxlen] = '\0';

		p = strstr((char *) mod->rxb, header);
		if (!p)
			continue;

		end = strstr(p, ending);
		if (!end)
			continue;

		p = strstr(p, ",");
		if (!p)
			return -1;

		p++;
		if (!*p)
			return -1;

		status = strtoul(p, &p, 0);
		if (!*p)
			return -1;

		p++;
		if (!*p)
			return -1;

		if (len)
			*len = strtoul(p, NULL, 0);

		return status;
	}

	return -1;
}

// \r\n+HTTPHEAD: 224
// \r\nhttp/1.1 200 ok
// \r\nserver: nginx/1.18.0 (ubuntu)
// \r\ndate: mon, 06 jan 2025 11:42:37 gmt
// \r\ncontent-type: application/json
// \r\ncontent-length: 32
// \r\nconnection: keep-alive
// \r\nauthorization: 93bsl2iertjhmgypran0jhssj3lxke66shih2qx4hqg=
// \r\n\r\n\r\nOK\r\n
// @retval: "Authorization" token length on success or -1 on failure
static int parse_http_head_auth(struct sim800l *mod, timeout_t timeout)
{
	struct sim800l_http *data = mod->task.data;
	const char *header = "authorization:";
	const char *ending = "\r\n";
	char *p, *end;
	int len;

	while (wait_for_any(mod, timeout))
	{
		mod->rxb[mod->rxlen] = '\0';

		p = strstr((char *) mod->rxb, header);
		if (!p)
			continue;

		end = strstr(p, ending);
		if (!end)
			continue;

		p = strstr(p, " ");
		if (!p)
			return -1;

		p++;
		if (!*p)
			return -1;

		if (p >= end)
			return -1;

		len = end - p;
		data->res_auth = pvPortMalloc(len + 1);
		if (!data->res_auth)
			return -1;

		memcpy(data->res_auth, p, len);
		data->res_auth[len] = '\0';

		return len;
	}

	return -1;
}

// \r\n+HTTPREAD: 293\r\n...\r\nOK\r\n
// @retval: Payload length on success or -1 on failure
static int parse_http_read(struct sim800l *mod, timeout_t timeout)
{
	struct sim800l_http *data = mod->task.data;
	const char *header = "+HTTPREAD:";
	const char *ending = "\r\n"; // To be sure len is valid
	char *p, *end;
	int len;

	while (wait_for_any(mod, timeout))
	{
		mod->rxb[mod->rxlen] = '\0';

		p = strstr((char *) mod->rxb, header);
		if (!p)
			continue;

		end = strstr(p, ending);
		if (!end)
			continue;

		p = strstr(p, " ");
		if (!p)
			return -1;

		p++;
		if (!*p)
			return -1;

		len = strtoul(p, &p, 0);
		if (!*p)
			return -1;

		p = end + strlen(ending);
		end = (char *) &mod->rxb[mod->rxlen];
		if (p > end)
			continue;

		if ((end - p) < len)
			continue;

		data->response = pvPortMalloc(len + 1);
		if (!data->response)
			return -1;

		memcpy(data->response, p, len);
		data->response[len] = '\0';
		data->rlen = len;

		return len;
	}

	return -1;
}

// \r\n+HTTPSTATUS: POST,0,0,0\r\n\r\nOK\r\n
// \r\nOK\r\n\r\n+HTTPSTATUS: GET,0,0,0\r\n\r\nOK\r\n
// @retval: HTTP status on success or -1 on failure
static int parse_http_status(struct sim800l *mod, timeout_t timeout)
{
	const char *header = "+HTTPSTATUS:";
	const char *ending = "\r\nOK\r\n";
	char *p, *end;
	int status;

	while (wait_for_any(mod, timeout))
	{
		mod->rxb[mod->rxlen] = '\0';

		p = strstr((char *) mod->rxb, header);
		if (!p)
			continue;

		end = strstr(p, ending);
		if (!end)
			continue;

		p = strstr(p, ",");
		if (!p)
			return false;

		p++;
		if (!*p)
			return false;

		status = strtoul(p, NULL, 0);

		return status;
	}

	return -1;
}

static int32_t get_param_value(const char *line, const char *param, int base)
{
	char *p = strstr(line, param);
	if (!p)
		return -1;

	p += strlen(param);
	if (!*p)
		return -1;

	p++; // ':'
	if (!*p)
		return -1;

	return strtoul(p, NULL, base);
}

static void shift_buffer_left(struct sim800l *mod, size_t len)
{
	uint8_t *p = mod->rxb + len;

	for (size_t i = 0; i < mod->rxlen - len; i++)
		mod->rxb[i] = p[i];
	mod->rxlen -= len;
}

static int parse_net_scan(struct sim800l *mod, timeout_t timeout)
{
	struct sim800l_netscan *data = mod->task.data;
	const char *ending = "OK\r\n";
	const char *delim = "\r\n";
	size_t len;
	char *p;

	while (wait_for_any(mod, timeout))
	{
		for (;;)
		{
			mod->rxb[mod->rxlen] = '\0';

			p = strstr((char *) mod->rxb, delim);
			if (!p)
				break; /* for */

			p += strlen(delim);
			len = p - (char *) mod->rxb;

			if (!strncmp((char *) mod->rxb, ending, strlen(ending)))
			{
				mod->task.callback(SIM800L_NETSCAN_DONE, mod->task.data);
				return 0;
			}

			// "\r\n" only
			if (len == strlen(delim))
			{
				shift_buffer_left(mod, len);
				continue; /* for */
			}

			// Parameters line
			mod->rxb[len - 1] = '\0';

			// Parameters
			data->mcc = get_param_value((char *) mod->rxb, "MCC", 10);
			data->mnc = get_param_value((char *) mod->rxb, "MNC", 10);
			data->lac = get_param_value((char *) mod->rxb, "Lac", 16);
			data->cid = get_param_value((char *) mod->rxb, "Cellid", 16);
			data->lev = get_param_value((char *) mod->rxb, "Rxlev", 10);

			if (data->mcc >= 0 && data->mnc >= 0 && data->lac >= 0 &&
					data->cid >= 0 && data->lev >= 0)
			{
				data->lev = data->lev - 113;
				mod->task.callback(0, mod->task.data);
			}

			// <--
			shift_buffer_left(mod, len);
		}
	}

	return -1;
}

inline static void upd_voltage_data(struct sim800l *mod)
{
	struct sim800l_voltage *data = mod->task.data;
	data->voltage = mod->voltage;
}

inline static char *get_http_url(struct sim800l *mod)
{
	struct sim800l_http *data = mod->task.data;
	return data->url;
}

static int get_http_method(struct sim800l *mod)
{
	struct sim800l_http *data = mod->task.data;
	if (!data->request)
		return 0; /* GET */
	else
		return 1; /* POST */
}

inline static size_t get_http_request_len(struct sim800l *mod)
{
	struct sim800l_http *data = mod->task.data;
	if (!data->request)
		return 0;
	return strlen(data->request);
}

inline static char *get_http_request(struct sim800l *mod)
{
	struct sim800l_http *data = mod->task.data;
	return data->request;
}

inline static char *get_http_req_auth(struct sim800l *mod)
{
	struct sim800l_http *data = mod->task.data;
	return data->req_auth;
}

inline static bool get_http_res_auth_get(struct sim800l *mod)
{
	struct sim800l_http *data = mod->task.data;
	return data->res_auth_get;
}

/******************************************************************************/
void sim800l_task(struct sim800l *mod)
{
	char cmd[CMD_BUFFER_SIZE];
	TickType_t ticks;
	bool done;

	for (;;)
	{
		switch ((enum state) mod->state)
		{
		case STATE_STARTUP:
			state(mod, STATE_RESET, STATUS_OK);
			break;

		case STATE_RESET:
			reset_set(mod);
			osDelay(200);
			reset_unset(mod);
			osDelay(3000);

			state(mod, STATE_AT, STATUS_OK);
			break;

		case STATE_AT:
			ticks = xTaskGetTickCount();
			done = false;

			while ((xTaskGetTickCount() - ticks) < pdMS_TO_TICKS(2000))
			{
				transmit(mod, "AT");
				if (compare_buffer_beginning(mod, "AT\r\r\nOK\r\n", 500))
				{
					state(mod, STATE_FLUSH, STATUS_OK);
					done = true;
					break; /* while */
				}
			}
			if (!done)
				state(mod, STATE_STARTUP, STATUS_ERROR);
			break;

		case STATE_FLUSH:
			// TODO: Wait for something?
			// "AT\r\r\nOK\r\n\r\nRDY\r\n\r\n+CFUN: 1\r\n\r\n+CPIN: READY\r\n"
			wait_for_num(mod, SIM800L_BUFFER_SIZE, 3000);
			state(mod, STATE_ECHO_OFF, STATUS_OK);
			break;

		case STATE_ECHO_OFF:
			transmit(mod, "ATE0");
			if (compare_buffer_beginning(mod, "ATE0\r\r\nOK\r\n", 500))
				state(mod, STATE_DEL_SMS, STATUS_OK);
			else
				state(mod, STATE_STARTUP, STATUS_ERROR);
			break;

		case STATE_DEL_SMS:
			// TODO: ?
			transmit(mod, "AT+CMGDA=6");
			if (compare_buffer_beginning(mod, "\r\nOK\r\n", 5000))
				state(mod, STATE_IDLE, STATUS_OK);
			else if (compare_buffer_beginning(mod, "\r\nERROR\r\n", 1))
				state(mod, STATE_IDLE, STATUS_OK);
			else
				state(mod, STATE_STARTUP, STATUS_ERROR);
			break;

		case STATE_IDLE:
			// Previous task: try again
			if (mod->task.issue != ISSUE_IDLE)
			{
				state(mod, STATE_DO_TASK, STATUS_OK);
				break;
			}

			// New task
			if (uxQueueMessagesWaiting(mod->queue))
			{
				xQueueReceive(mod->queue, &mod->task, 0);
				mod->task_ticks = xTaskGetTickCount();

				state(mod, STATE_DO_TASK, STATUS_OK);
				break;
			}

			// No new task: sleep and wait
#ifdef DISABLE_RF
			transmit(mod, "AT+CFUN=0");
			if (!compare_buffer_beginning(mod,
					"\r\n+CPIN: NOT READY\r\n\r\nOK\r\n", 10000))
			{
				state(mod, STATE_STARTUP, STATUS_ERROR);
				break;
			}
#endif /* DISABLE_RF */

			transmit(mod, "AT+CSCLK=2");
			if (!compare_buffer_beginning(mod, "\r\nOK\r\n", 500))
			{
				state(mod, STATE_STARTUP, STATUS_ERROR);
				break;
			}

#ifdef LOGGER
			logger_add_str(&logger, TAG, false, "sleep...");
#endif

			// Wait for the task
			xQueueReceive(mod->queue, &mod->task, portMAX_DELAY);
			mod->task_ticks = xTaskGetTickCount();

			transmit(mod, "AT");
			osDelay(150);

			transmit(mod, "AT+CSCLK=0");
			if (!compare_buffer_beginning(mod, "\r\nOK\r\n", 500))
			{
				state(mod, STATE_STARTUP, STATUS_ERROR);
				break;
			}

#ifdef DISABLE_RF
			transmit(mod, "AT+CFUN=1");
			if (!compare_buffer_beginning(mod,
					"\r\n+CPIN: READY\r\n\r\nOK\r\n\r\nSMS Ready\r\n", 10000))
			{
				state(mod, STATE_STARTUP, STATUS_ERROR);
				break;
			}
#endif /* DISABLE_RF */

			state(mod, STATE_DO_TASK, STATUS_OK);
			break;

		case STATE_DO_TASK:
			switch ((enum issue) mod->task.issue)
			{
			case ISSUE_IDLE:
				// Never be here
				osDelay(1);
				break;

			case ISSUE_VOLTAGE:
				state(mod, STATE_CBC, STATUS_OK);
				break;

			case ISSUE_HTTP:
				state(mod, STATE_CREG, STATUS_OK);
				break;

			case ISSUE_NETSCAN:
				state(mod, STATE_CREG, STATUS_OK);
				break;
			}
			break;

		case STATE_CBC:
			transmit(mod, "AT+CBC");
			if (parse_battery_charge(mod, 500))
			{
				upd_voltage_data(mod);
				mod->task.callback(0, mod->task.data);
				task_done(mod);
				state(mod, STATE_IDLE, STATUS_OK);
			}
			else
			{
				state(mod, STATE_CBC, STATUS_ERROR);
			}
			break;

		case STATE_CREG:
			ticks = xTaskGetTickCount();
			done = false;

			while ((xTaskGetTickCount() - ticks) < pdMS_TO_TICKS(30000))
			{
				transmit(mod, "AT+CREG?");
				if (compare_buffer_beginning(mod,
						"\r\n+CREG: 0,1\r\n\r\nOK\r\n", 5000))
				{
					done = true;
					break; /* while */
				}
				osDelay(1000);
			}
			if (!done)
			{
				state(mod, STATE_IDLE, STATUS_ERROR);
				break;
			}

			if (mod->task.issue == ISSUE_NETSCAN)
				state(mod, STATE_NETSCAN, STATUS_OK);
			else /* ISSUE_HTTP */
				state(mod, STATE_GPRS_INIT, STATUS_OK);
			break;

		case STATE_GPRS_INIT:
			transmit(mod, "AT+SAPBR=3,1,CONTYPE,GPRS");
			if (!compare_buffer_beginning(mod, "\r\nOK\r\n", 500))
			{
				state(mod, STATE_IDLE, STATUS_ERROR);
				break;
			}

			strcpy(cmd, "AT+SAPBR=3,1,APN,");
			strcat(cmd, mod->apn);
			transmit(mod, cmd);
			if (!compare_buffer_beginning(mod, "\r\nOK\r\n", 500))
			{
				state(mod, STATE_IDLE, STATUS_ERROR);
				break;
			}

			transmit(mod, "AT+SAPBR=1,1");
			if (!compare_buffer_beginning(mod, "\r\nOK\r\n", 20000))
			{
				state(mod, STATE_IDLE, STATUS_ERROR);
				break;
			}

			// TODO: Copy IP
			// "\r\n+SAPBR: 1,1,\"10.68.222.113\"\r\n\r\nOK\r\n"
			transmit(mod, "AT+SAPBR=2,1");
			if (!compare_buffer_beginning(mod, "\r\n+SAPBR: 1,1", 20000))
			{
				state(mod, STATE_IDLE, STATUS_ERROR);
				break;
			}

			state(mod, STATE_GPRS_HTTP, STATUS_OK);
			break;

		case STATE_GPRS_HTTP:
			transmit(mod, "AT+HTTPINIT");
			if (!compare_buffer_beginning(mod, "\r\nOK\r\n", 2000))
			{
				state(mod, STATE_IDLE, STATUS_ERROR);
				break;
			}

			transmit(mod, "AT+HTTPPARA=CID,1");
			if (!compare_buffer_beginning(mod, "\r\nOK\r\n", 2000))
			{
				state(mod, STATE_IDLE, STATUS_ERROR);
				break;
			}

			strcpy(cmd, "AT+HTTPPARA=URL,");
			strcat(cmd, get_http_url(mod));
			transmit(mod, cmd);
			if (!compare_buffer_beginning(mod, "\r\nOK\r\n", 2000))
			{
				state(mod, STATE_IDLE, STATUS_ERROR);
				break;
			}

			// SSL does not work
			// \r\nOK\r\n\r\n+HTTPACTION: 0,606,0\r\n

			// Request "Authorization" header
			if (get_http_req_auth(mod))
			{
				strcpy(cmd, "AT+HTTPPARA=USERDATA,Authorization: ");
				strcat(cmd, get_http_req_auth(mod));
				transmit(mod, cmd);
				if (!compare_buffer_beginning(mod, "\r\nOK\r\n", 2000))
				{
					state(mod, STATE_IDLE, STATUS_ERROR);
					break;
				}
			}

			// HTTP POST
			if (get_http_method(mod))
			{
				transmit(mod, "AT+HTTPPARA=CONTENT,application/json");
				if (!compare_buffer_beginning(mod, "\r\nOK\r\n", 500))
				{
					state(mod, STATE_IDLE, STATUS_ERROR);
					break;
				}

				strcpy(cmd, "AT+HTTPDATA=");
				utoa(get_http_request_len(mod), &cmd[strlen(cmd)], 10);
				strcat(cmd, ",1000");
				transmit(mod, cmd);
				if (!compare_buffer_beginning(mod, "\r\nDOWNLOAD\r\n", 1000))
				{
					state(mod, STATE_IDLE, STATUS_ERROR);
					break;
				}

				transmit(mod, get_http_request(mod));
				if (!compare_buffer_beginning(mod, "\r\nOK\r\n", 500))
				{
					state(mod, STATE_IDLE, STATUS_ERROR);
					break;
				}
			}

			strcpy(cmd, "AT+HTTPACTION=");
			if (!get_http_method(mod))
				strcat(cmd, "0"); /* GET */
			else
				strcat(cmd, "1"); /* POST */
			transmit(mod, cmd);
			if (parse_http_action(mod, NULL, 30000) != 200)
			{
				state(mod, STATE_GPRS_HTTP_TERM, STATUS_ERROR);
				break;
			}

			// Response "Authorization" header
			if (get_http_res_auth_get(mod))
			{
				transmit(mod, "AT+HTTPHEAD");
				parse_http_head_auth(mod, 1000);
			}

			transmit(mod, "AT+HTTPREAD");
			if (parse_http_read(mod, 1000) > 0)
			{
				mod->task.callback(0, mod->task.data);
				task_done(mod);
			}

			state(mod, STATE_GPRS_HTTP_TERM, STATUS_OK);
			break;

		case STATE_GPRS_HTTP_TERM:
			// TODO: Check in while loop with timeout
			transmit(mod, "AT+HTTPSTATUS?");
			if (parse_http_status(mod, 500))
			{
				state(mod, STATE_GPRS_DEINIT, STATUS_ERROR);
				break;
			}

			transmit(mod, "AT+HTTPTERM");
			if (!compare_buffer_beginning(mod, "\r\nOK\r\n", 2000))
			{
				state(mod, STATE_GPRS_DEINIT, STATUS_ERROR);
				break;
			}

			// Get the next task now
			// Continue to send HTTP while connected to the Internet
			if (mod->task.issue == ISSUE_IDLE)
			{
				// 50ms for tasks to create a new HTTP request
				if (xQueueReceive(mod->queue, &mod->task, pdMS_TO_TICKS(50)))
				{
					mod->task_ticks = xTaskGetTickCount();
					if (mod->task.issue == ISSUE_HTTP)
					{
						state(mod, STATE_GPRS_HTTP, STATUS_OK);
						break;
					}
				}
			}

			state(mod, STATE_GPRS_DEINIT, STATUS_OK);
			break;

		case STATE_GPRS_DEINIT:
			transmit(mod, "AT+SAPBR=0,1");
			if (compare_buffer_beginning(mod, "\r\nOK\r\n", 10000))
				state(mod, STATE_IDLE, STATUS_OK);
			else
				state(mod, STATE_IDLE, STATUS_ERROR);
			break;

		case STATE_NETSCAN:
			transmit(mod, "AT+CNETSCAN=1");
			if (!compare_buffer_beginning(mod, "\r\nOK\r\n", 500))
			{
				state(mod, STATE_IDLE, STATUS_ERROR);
				break;
			}

			transmit(mod, "AT+CNETSCAN");
			if(!parse_net_scan(mod, 45000)) // Callbacks inside
			{
				task_done(mod);
				state(mod, STATE_IDLE, STATUS_OK);
			}
			break;
		}
	}
}

/******************************************************************************/
int sim800l_voltage(struct sim800l *mod, struct sim800l_voltage *data,
		sim800l_cb callback, timeout_t timeout)
{
	struct sim800l_task task;
	BaseType_t status;

	task.issue = ISSUE_VOLTAGE;
	task.timeout = pdMS_TO_TICKS(timeout);
	task.callback = callback;
	task.data = data;

	status = xQueueSendToBack(mod->queue, &task, 0);

	if (status == pdTRUE)
		return 0;

	return -1;
}

/******************************************************************************/
int sim800l_http(struct sim800l *mod, struct sim800l_http *data,
		sim800l_cb callback, timeout_t timeout)
{
	struct sim800l_task task;
	BaseType_t status;

	data->response = NULL;
	data->res_auth = NULL;

	task.issue = ISSUE_HTTP;
	task.timeout = pdMS_TO_TICKS(timeout);
	task.callback = callback;
	task.data = data;

	status = xQueueSendToBack(mod->queue, &task, 0);

	if (status == pdTRUE)
		return 0;

	return -1;
}

/******************************************************************************/
int sim800l_netscan(struct sim800l *mod, struct sim800l_netscan *data,
		sim800l_cb callback, timeout_t timeout)
{
	struct sim800l_task task;
	BaseType_t status;

	task.issue = ISSUE_NETSCAN;
	task.timeout = pdMS_TO_TICKS(timeout);
	task.callback = callback;
	task.data = data;

	status = xQueueSendToBack(mod->queue, &task, 0);

	if (status == pdTRUE)
		return 0;

	return -1;
}
