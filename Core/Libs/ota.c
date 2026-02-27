/*
 * Over-the-air (OTA) firmware update
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#include "ota.h"

#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define JSMN_HEADER
#include "jsmn.h"

#include "sim800l.h"
#include "params.h"
#include "fws.h"

#define DELAY_HTTP_MS 60000
#define DELAY_SIM800L_MS (DELAY_HTTP_MS * 5)

#define JSON_MAX_TOKENS 32

//#define FILE_PART_SIZE (1024 - 128) // TODO: Not enough RAM
#define FILE_PART_SIZE 512
#define FLASH_WRITE_SIZE 128 // Not 256 because of FILE_PART_SIZE value
#define FLASH_READ_SIZE 32

#define RETRIES 5

extern const uint32_t *_app_len;
#define APP_LENGTH ((uint32_t) &_app_len)


/******************************************************************************/
void ota_init(struct ota *ota, struct sim800l *mod, struct w25q_s *mem,
		const uint8_t *secret, const char *url)
{
	memset(ota, 0, sizeof(*ota));
	ota->mod = mod;
	ota->mem = mem;
	memcpy(ota->secret, secret, sizeof(ota->secret));
	strncpy(ota->url, url, sizeof(ota->url) - 1);
	ota->url[sizeof(ota->url) - 1] = '\0';
}

// https://github.com/zserge/jsmn/blob/master/example/simple.c
static int jsoneq(const char *json, jsmntok_t *tok, const char *s)
{
	if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
			strncmp(json + tok->start, s, tok->end - tok->start) == 0)
		return 0;
	return -1;
}

static void jsoncpy(const char *json, jsmntok_t *tok, char *dst, size_t mlen)
{
	size_t len = tok->end - tok->start;
	if (len > mlen - 1)
		len = mlen - 1;
	strncpy(dst, json + tok->start, len);
	dst[len] = '\0';
}

static int parse_json(const char *response, struct fws *fws, char *filename,
		size_t mlen)
{
	jsmn_parser parser;
	jsmntok_t tokens[JSON_MAX_TOKENS];
	jsmntok_t *object;
	jsmntok_t *token_file, *token_ver, *token_cs, *token_size;
	uint32_t vermax = 0;
	int i, ret;

	memset(tokens, 0, sizeof(tokens));
	jsmn_init(&parser);
	ret = jsmn_parse(&parser, response, strlen(response), tokens,
			JSON_MAX_TOKENS);

	if (ret <= 0)
		return -1;

	if (tokens[0].type != JSMN_ARRAY)
		return -1;

	i = 1;
	for (int objs = 0; objs < tokens[0].size && i < ret; objs++)
	{
		object = &tokens[i];
		if (object->type != JSMN_OBJECT)
			return -1;

		i++;
		token_file = NULL;
		token_ver = NULL;
		token_cs = NULL;
		token_size = NULL;
		for (int items = 0; items < object->size && (i + 1) < ret; items++)
		{
			if (jsoneq(response, &tokens[i], "file") == 0)
				token_file = &tokens[i + 1];
			else if (jsoneq(response, &tokens[i], "ver") == 0)
				token_ver = &tokens[i + 1];
			else if (jsoneq(response, &tokens[i], "checksum") == 0)
				token_cs = &tokens[i + 1];
			else if (jsoneq(response, &tokens[i], "size") == 0)
				token_size = &tokens[i + 1];

			i += 2;
		}

		if (token_file && token_ver && token_cs && token_size)
		{
			char temp[16];
			uint32_t ver;

			jsoncpy(response, token_ver, temp, sizeof(temp));
			ver = strtoul(temp, NULL, 0);

			if (ver > vermax)
			{
				vermax = ver;

				fws->loaded = 0;
				fws->version = ver;
				jsoncpy(response, token_cs, temp, sizeof(temp));
				fws->checksum = strtoul(temp, NULL, 0);
				jsoncpy(response, token_size, temp, sizeof(temp));
				fws->size = strtoul(temp, NULL, 0);

				jsoncpy(response, token_file, filename, mlen);
			}
		}
	}

	return vermax;
}

static void callback(int status, void *data)
{
	struct sim800l_http *http = data;
	TaskHandle_t task = *((TaskHandle_t *) http->context);

	BaseType_t woken = pdFALSE;
	vTaskNotifyGiveFromISR(task, &woken);
}

static int request_list(struct ota *ota, struct sim800l_http *http)
{
	strcpy(http->url, ota->url);
	strcat(http->url, "/api/list?name=");
	strcat(http->url, PARAMS_DEVICE_NAME);
	http->req_auth = NULL;
	http->res_auth = NULL; // Unnecessary
	http->res_auth_get = true;
	http->request = NULL;
	http->response = NULL; // Unnecessary
	http->context = &ota->task;

	return sim800l_http(ota->mod, http, callback, DELAY_HTTP_MS);
}

static int request_file(struct ota *ota, struct sim800l_http *http,
		const char *filename, uint32_t addr, uint32_t size)
{
	strcpy(http->url, ota->url);
	strcat(http->url, "/api/file?file=");
	strcat(http->url, filename);
	strcat(http->url, "&addr=");
	utoa(addr, &http->url[strlen(http->url)], 10);
	strcat(http->url, "&size=");
	utoa(size, &http->url[strlen(http->url)], 10);
	http->req_auth = NULL;
	http->res_auth = NULL; // Unnecessary
	http->res_auth_get = true;
	http->request = NULL;
	http->response = NULL; // Unnecessary
	http->context = &ota->task;

	return sim800l_http(ota->mod, http, callback, DELAY_HTTP_MS);
}

static void mem_erase(struct w25q_s *mem, uint32_t addr, uint16_t size)
{
	while (size)
	{
		w25q_s_sector_erase(mem, addr);

		if (size > W25Q_SECTOR_SIZE)
			size -= W25Q_SECTOR_SIZE;
		else
			size = 0;
		addr += W25Q_SECTOR_SIZE;
	}
}

static void mem_write(struct w25q_s *mem, uint32_t addr, uint8_t *buf,
		uint16_t size)
{
	uint16_t ws;

	while (size)
	{
		if (size > FLASH_WRITE_SIZE)
			ws = FLASH_WRITE_SIZE;
		else
			ws = size;

		w25q_s_write_data(mem, addr, buf, ws);

		size -= ws;
		addr += ws;
		buf += ws;
	}
}

static void mem_write_header(struct w25q_s *mem, struct fws *fws)
{
	w25q_s_sector_erase(mem, FWS_HEADER_ADDR);
	mem_write(mem, FWS_HEADER_ADDR, (uint8_t *) fws, sizeof(struct fws));
}

static uint32_t mem_checksum(struct w25q_s *mem, uint32_t addr, uint32_t size)
{
	uint32_t checksum = FWS_CHECKSUM_INIT;
	uint8_t buf[FLASH_READ_SIZE];
	uint32_t ws;

	while (size)
	{
		if (size > FLASH_READ_SIZE)
			ws = FLASH_READ_SIZE;
		else
			ws = size;

		w25q_s_read_data(mem, addr, buf, ws);

		size -= ws;
		addr += ws;

		for (uint32_t i = 0; i < ws; i += sizeof(uint32_t))
			checksum += *((uint32_t *) &buf[i]);
	}

	return checksum;
}

static void strtolower(char *data)
{
	while (*data)
	{
		*data = tolower(*data);
		data++;
	}
}

/******************************************************************************/
void ota_task(struct ota *ota)
{
	struct sim800l_http http;
	char filename[64];
	struct fws fws;
	uint32_t addr;
	int retries;
	int ret;

	char url[OTA_URL_SIZE + 64];
	char hmac[HMAC_BASE64_LEN];

	if (w25q_s_get_manufacturer_id(ota->mem) != FWS_WINBOND_MANUFACTURER_ID)
		vTaskDelete(NULL);

	ota->task = xTaskGetCurrentTaskHandle();
	http.url = url;

	goto startup;

	for (;;)
	{
		osDelay(OTA_CHECK_INTERVAL_MS);

startup:
		// Request firmware list
		ret = request_list(ota, &http);
		if (ret)
			continue;
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(DELAY_SIM800L_MS));

		// Error
		if (!http.res_auth || !http.response)
		{
			if (http.res_auth)
				vPortFree(http.res_auth);
			if (http.response)
				vPortFree(http.response);
			continue;
		}

		// Authorization
		hmac_base64(ota->secret, http.response, http.rlen, hmac);
		strtolower(hmac);
		if (strcmp(hmac, http.res_auth) != 0)
		{
			vPortFree(http.res_auth);
			vPortFree(http.response);
			continue;
		}

		// Parse firmware list
		ret = parse_json(http.response, &fws, filename, sizeof(filename));
		vPortFree(http.res_auth);
		vPortFree(http.response);

		// No update or error
		if (ret <= PARAMS_FW_VERSION)
			continue;

		// Too big firmware
		if (fws.size > APP_LENGTH)
			continue;

		// Erase SPI FLASH
		mem_erase(ota->mem, FWS_PAYLOAD_ADDR, fws.size);

		// Updating
		addr = 0;
		retries = RETRIES;
		while (retries && addr < fws.size)
		{
			// Request newest firmware file
			ret = request_file(ota, &http, filename, addr, FILE_PART_SIZE);
			if (ret)
			{
				retries--;
				continue; /* while */
			}
			ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(DELAY_SIM800L_MS));

			// Error
			if (!http.res_auth || !http.response)
			{
				if (http.res_auth)
					vPortFree(http.res_auth);
				if (http.response)
					vPortFree(http.response);
				retries--;
				continue; /* while */
			}

			// Authorization
			hmac_base64(ota->secret, http.response, http.rlen, hmac);
			strtolower(hmac);
			if (strcmp(hmac, http.res_auth) != 0)
			{
				vPortFree(http.res_auth);
				vPortFree(http.response);
				retries--;
				continue; /* while */
			}

			// Write data to SPI FLASH
			mem_write(ota->mem, FWS_PAYLOAD_ADDR + addr,
					(uint8_t *) http.response, http.rlen);

			addr += http.rlen;
			retries = RETRIES; // Reset retries

			vPortFree(http.res_auth);
			vPortFree(http.response);
		}

		// Expand to a multiple of 4
		if (fws.size % sizeof(uint32_t))
		{
			int expand = sizeof(uint32_t) - fws.size % sizeof(uint32_t);
			uint8_t byte = 0xFF;

			fws.size += expand;
			while (expand)
			{
				mem_write(ota->mem, FWS_PAYLOAD_ADDR + addr, &byte, 1);
				expand--;
				addr++;

			}
		}

		// Checksum
		if (mem_checksum(ota->mem, FWS_PAYLOAD_ADDR, fws.size) != fws.checksum)
			continue;

		// Update SPI FLASH header
		fws.loaded = 0;
		mem_write_header(ota->mem, &fws);

		// Reset
		osDelay(100); // ?
		NVIC_SystemReset(); // --> BOOTLOADER
	}
}
