/*
 * Application task
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#include "ptasks.h"

#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <ctype.h>

#define JSMN_HEADER
#include "jsmn.h"

#include "strjson.h"
#include "base64.h"
#include "params.h"
#include "queue.h"
#include "main.h"
#include "hmac.h"
#include "fws.h"

#include "logger.h"
#ifdef LOGGER
#define TAG "APP"
extern struct logger logger;
#endif

#define JSON_MAX_TOKENS 8

#define MAX_QTY_FROM_QUEUE 4

#define NET_LEV_MIN -113

#define HTTP_TIMEOUT_1MIN 60000
#define HTTP_TIMEOUT_2MIN 120000

#define NETSCAN_TIMEOUT_1MIN 60000

#define TIME_UPDATE_PERIOD (24 * 60 * 60 * 1000)

#define READ_TAMPER (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)) // TODO

static osThreadId_t handle;
static const osThreadAttr_t attributes = {
  .name = "app",
  .stack_size = 496 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

struct netprms
{
	int32_t mcc;
	int32_t mnc;
	int32_t lac;
	int32_t cid;
	int32_t lev;
};

struct sensor_item
{
	uint32_t value;
	uint32_t timestamp;
};

#define SENSORS_BUF_LEN (MAX_QTY_FROM_QUEUE * sizeof(struct sensor_item))
#define SENSORS_STR_LEN (4 * ((SENSORS_BUF_LEN + 2) / 3) + 1)

static void http_callback(int status, void *data)
{
	struct sim800l_http *http = data;
	BaseType_t woken = pdFALSE;

	*((int *) http->context) = status;

	vTaskNotifyGiveFromISR(handle, &woken);
}

static void netscan_callback(int status, void *data)
{
	struct sim800l_netscan *netscan = data;
	struct netprms *prms = netscan->context;
	BaseType_t woken = pdFALSE;

	if (netscan->lev > prms->lev)
	{
		prms->mcc = netscan->mcc;
		prms->mnc = netscan->mnc;
		prms->lac = netscan->lac;
		prms->cid = netscan->cid;
		prms->lev = netscan->lev;
	}

	if (status != 0)
		vTaskNotifyGiveFromISR(handle, &woken);
}

static void blink(void)
{
	HAL_GPIO_WritePin(LED_MB_GPIO_Port, LED_MB_Pin, GPIO_PIN_SET);
	osDelay(100);
	HAL_GPIO_WritePin(LED_MB_GPIO_Port, LED_MB_Pin, GPIO_PIN_RESET);
}

// https://github.com/zserge/jsmn/blob/master/example/simple.c
static int jsoneq(const char *json, jsmntok_t *tok, const char *s)
{
	if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
			strncmp(json + tok->start, s, tok->end - tok->start) == 0)
		return 0;
	return -1;
}

static int parse_json_time(const char *response, uint32_t *ts)
{
	jsmn_parser parser;
	jsmntok_t tokens[JSON_MAX_TOKENS];
	int ret;

	memset(tokens, 0, sizeof(tokens));
	jsmn_init(&parser);
	ret = jsmn_parse(&parser, response, strlen(response), tokens,
			JSON_MAX_TOKENS);

	if (ret <= 0)
		return -1;

	if (tokens[0].type != JSMN_OBJECT)
		return -1;

	for (int i = 1; (i + 1) < ret; i += 2)
	{
		if (jsoneq(response, &tokens[i], "ts") == 0)
			*ts = strtoul(response + tokens[i + 1].start, NULL, 0);
	}

	return 0;
}

static int sensor_read(mqueue_t *queue, struct sensor_record *recs, int max)
{
	int i = 0;
	int ret;

	for (;;)
	{
		if (i >= max)
			break;

		ret = mqueue_get(queue, &recs[i]);
		if (ret == -MFIFO_ERR_INVALID_CRC)
			continue;
		if (ret)
			break;

		i++;
	}

	return i;
}

static void sensor_field_base64(struct sensor_record *recs, int count,
		size_t offset, char *buf)
{
	struct sensor_item sensor_items[MAX_QTY_FROM_QUEUE];
	size_t enclen;

	for (int i = 0; i < count; i++)
	{
		sensor_items[i].timestamp = recs[i].timestamp;
		sensor_items[i].value = *(uint32_t *)((uint8_t *)&recs[i] + offset);
	}

	base64_encode((unsigned char *) sensor_items, sizeof(struct sensor_item) * count, buf,
			&enclen);
	buf[enclen] = '\0';
}

static void strtolower(char *data)
{
	while (*data)
	{
		*data = tolower(*data);
		data++;
	}
}

/**
 * @brief: Add HTTP GET request to SIM800L task queue
 * @info: http->url must be already allocated
 */
static int http_get(struct app *app, struct sim800l_http *http, const char *api)
{
	strcpy(http->url, app->params->url_app);
	strcat(http->url, api);
	http->req_auth = NULL; // Not used
	http->res_auth = NULL; // Unnecessary
	http->res_auth_get = true;
	http->request = NULL; // Not used
	http->response = NULL; // Unnecessary

	return sim800l_http(app->mod, http, http_callback, HTTP_TIMEOUT_2MIN);
}

/**
 * @brief: Add HTTP POST request to SIM800L task queue
 * @info: http->url must be already allocated
 * @info: http->req_auth must be already allocated
 * @info: http->request must be already allocated and filled with data
 */
static int http_post(struct app *app, struct sim800l_http *http,
		const char *api)
{
	strcpy(http->url, app->params->url_app);
	strcat(http->url, api);
	hmac_base64(app->params->secret, http->request, strlen(http->request),
				http->req_auth);
	http->res_auth = NULL; // Unnecessary
	http->res_auth_get = false;
	http->response = NULL; // Unnecessary

	return sim800l_http(app->mod, http, http_callback, HTTP_TIMEOUT_2MIN);
}

static int parse_time(struct app *app, struct sim800l_http *http,
		char *hmacbuf)
{
	uint32_t temp;
	int ret;

	if (!http->res_auth)
		return -1;

	// NOTE: Ignore case of characters
	hmac_base64(app->params->secret, http->response, http->rlen, hmacbuf);
	strtolower(hmacbuf);
	if (strcmp(hmacbuf, http->res_auth))
		return -1;

	ret = parse_json_time(http->response, &temp);
	if (ret)
		return -1;

	*app->timestamp = temp;
#ifdef LOGGER
	logger_add_str(&logger, TAG, false, http->response);
#endif

	return 0;
}

/**
 * @brief: Send HTTP GET request, wait and parse response
 * @info: http->url must be already allocated
 */
static int proc_http_get_time(struct app *app, struct sim800l_http *http,
		char *hmacbuf)
{
	int status;
	int ret;

	http->context = &status;

	ret = http_get(app, http, "/api/time");
	if (ret)
		return -1;

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	if (!status)
		ret = parse_time(app, http, hmacbuf);

	if (http->res_auth)
		vPortFree(http->res_auth);
	if (http->response)
		vPortFree(http->response);

	if (status || ret)
		return -1;

	return 0;
}

/**
 * @brief: Send HTTP POST request and wait for response
 * @info: http->url must be already allocated
 * @info: http->req_auth must be already allocated
 * @info: http->request must be already allocated and filled with data
 */
	static int proc_http_post(struct app *app, struct sim800l_http *http,
		const char *api)
{
	int status;
	int ret;

	http->context = &status;

	ret = http_post(app, http, api);
	if (ret)
		return -1;

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	if (http->response)
		vPortFree(http->response);

	if (status)
		return -1;

	return 0;
}

static void task(void *argument)
{
	struct app *app = argument;

	struct sim800l_netscan netscan;
	struct sim800l_http get, post;
	struct netprms netprms;
	TickType_t updt;
	int voltage;
	int avail;
	int ret;

	char url[PARAMS_APP_URL_SIZE + 32]; // Same for post and get
	char hmac[HMAC_BASE64_LEN];
	char request[512]; // NOTE: ?

	// post buffers
	post.url = url;
	post.req_auth = hmac;
	post.request = request;

	// get buffers
	get.url = url;

	// netscan
	memset(&netprms, 0, sizeof(netprms));
	netprms.lev = NET_LEV_MIN;
	netscan.context = &netprms;

	updt = xTaskGetTickCount();

	// >>>

	// <- /api/time
	while (proc_http_get_time(app, &get, hmac))
		xEventGroupWaitBits(sync_events, SYNC_BIT_APP,
				pdTRUE, pdFALSE, portMAX_DELAY);

	// Available sensors
	xSemaphoreTake(app->sens->actual->mutex, portMAX_DELAY);
	avail = app->sens->actual->avail;
	xSemaphoreGive(app->sens->actual->mutex);

	// -> /api/info
	strjson_init(request);
	strjson_uint(request, "uid", app->params->id);
	strjson_uint(request, "ts", *app->timestamp);
	strjson_str(request, "name", PARAMS_DEVICE_NAME);
	strjson_str(request, "bl_git", (char *) app->bl->hash);
	strjson_uint(request, "bl_status", app->bl->status);
	strjson_str(request, "app_git", GIT_COMMIT_HASH);
	strjson_uint(request, "app_ver", PARAMS_FW_VERSION);
	strjson_str(request, "mcu", app->params->mcu_uid);
	strjson_str(request, "apn", app->params->apn);
	strjson_str(request, "url_ota", app->params->url_ota);
	strjson_str(request, "url_app", app->params->url_app);
	strjson_uint(request, "period_app", app->params->period_app);
	strjson_uint(request, "period_sen", app->params->period_sen);
	strjson_uint(request, "mtime_count", app->params->mtime_count);
	strjson_int(request, "sens", avail);

	while (proc_http_post(app, &post, "/api/info"))
		xEventGroupWaitBits(sync_events, SYNC_BIT_APP,
				pdTRUE, pdFALSE, portMAX_DELAY);

	// netscan
	ret = sim800l_netscan(app->mod, &netscan, netscan_callback,
			NETSCAN_TIMEOUT_1MIN);
	if (!ret)
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	// -> /api/cnet
	strjson_init(request);
	strjson_uint(request, "uid", app->params->id);
	strjson_uint(request, "ts", *app->timestamp);
	strjson_int(request, "mcc", netprms.mcc);
	strjson_int(request, "mnc", netprms.mnc);
	strjson_int(request, "lac", netprms.lac);
	strjson_int(request, "cid", netprms.cid);
	strjson_int(request, "lev", netprms.lev);

	while (proc_http_post(app, &post, "/api/cnet"))
		xEventGroupWaitBits(sync_events, SYNC_BIT_APP,
				pdTRUE, pdFALSE, portMAX_DELAY);

	for (;;)
	{
		if ((xTaskGetTickCount() - updt) >= TIME_UPDATE_PERIOD)
		{
			updt = xTaskGetTickCount();

			// <- /api/time
			while (proc_http_get_time(app, &get, hmac))
				xEventGroupWaitBits(sync_events, SYNC_BIT_APP,
						pdTRUE, pdFALSE, portMAX_DELAY);
		}

		// Voltage
		xSemaphoreTake(app->sens->actual->mutex, portMAX_DELAY);
		voltage = app->sens->actual->voltage;
		xSemaphoreGive(app->sens->actual->mutex);

		// -> /api/data
		strjson_init(request);
		strjson_uint(request, "uid", app->params->id);
		strjson_uint(request, "ts", *app->timestamp);
		strjson_uint(request, "ticks", xTaskGetTickCount());
		if (voltage) {
			strjson_int(request, "bat", voltage);
		}
		{
			struct sensor_record recs[MAX_QTY_FROM_QUEUE];
			char sensor_records_base64[SENSORS_STR_LEN];
			int nrecs;

			nrecs = sensor_read(app->sens->queue, recs, MAX_QTY_FROM_QUEUE);

			// sensor_field_base64(recs, nrecs,
			// 		offsetof(struct sensor_record, voltage), sensor_records_base64);
			// if (*sensor_records_base64)
			// 	strjson_str(request, "bat", sensor_records_base64);
			sensor_field_base64(recs, nrecs,
					offsetof(struct sensor_record, count_avg), sensor_records_base64);
			if (*sensor_records_base64)
				strjson_str(request, "count", sensor_records_base64);
			sensor_field_base64(recs, nrecs,
					offsetof(struct sensor_record, count_min), sensor_records_base64);
			if (*sensor_records_base64)
				strjson_str(request, "count_min", sensor_records_base64);
			sensor_field_base64(recs, nrecs,
					offsetof(struct sensor_record, count_max), sensor_records_base64);
			if (*sensor_records_base64)
				strjson_str(request, "count_max", sensor_records_base64);
			sensor_field_base64(recs, nrecs,
					offsetof(struct sensor_record, temperature), sensor_records_base64);
			if (*sensor_records_base64)
				strjson_str(request, "temp", sensor_records_base64);
			sensor_field_base64(recs, nrecs,
					offsetof(struct sensor_record, humidity), sensor_records_base64);
			if (*sensor_records_base64)
				strjson_str(request, "hum", sensor_records_base64);
			sensor_field_base64(recs, nrecs,
					offsetof(struct sensor_record, angle), sensor_records_base64);
			if (*sensor_records_base64)
				strjson_str(request, "angle", sensor_records_base64);
		}
		strjson_int(request, "tamper", READ_TAMPER);

		while (proc_http_post(app, &post, "/api/data"))
			xEventGroupWaitBits(sync_events, SYNC_BIT_APP,
					pdTRUE, pdFALSE, portMAX_DELAY);

		if (!mqueue_is_empty(app->sens->queue))
			continue;

		blink();
		xEventGroupWaitBits(sync_events, SYNC_BIT_APP,
				pdTRUE, pdFALSE, portMAX_DELAY);
	}
}

/******************************************************************************/
void task_app(struct app *app)
{
	handle = osThreadNew(task, app, &attributes);
}
