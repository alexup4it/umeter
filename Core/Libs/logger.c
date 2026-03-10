/*
 * Logger
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2025
 */

#include "logger.h"

#include <stdlib.h>
#include <string.h>

#include "cmsis_os.h"
#include "queue.h"

#define MAX_DATA_LEN 96

/******************************************************************************/
void logger_init(struct logger* logger, struct siface* siface) {
    memset(logger, 0, sizeof(*logger));
    logger->siface = siface;
}

/******************************************************************************/
#ifdef LOGGER
int logger_add(struct logger* logger,
               const char* tag,
               bool full,
               const char* buf,
               size_t len) {
    char ticks[16];
    char* log;
    size_t ll;
    int ret;

    if (!full && len > MAX_DATA_LEN) {
        len = MAX_DATA_LEN;
    }

    utoa(xTaskGetTickCount(), ticks, 10);

    ll  = strlen(tag) + 1 + strlen(ticks) + 1 + len + 2 + 1;
    log = pvPortMalloc(ll);
    if (!log) {
        return -1;
    }

    strcpy(log, tag);
    strcat(log, ",");
    strcat(log, ticks);
    strcat(log, ",");
    memcpy(&log[strlen(log)], buf, len);
    log[ll - 3] = '\r';
    log[ll - 2] = '\n';
    log[ll - 1] = '\0';

    for (int i = 0; i < ll - 3; i++) {
        if (log[i] == '\r') {
            log[i] = 'r';
        } else if (log[i] == '\n') {
            log[i] = 'n';
        } else if ((log[i] < 0x20) || (log[i] > 0x7E)) {
            log[i] = '*';
        }
    }

    ret = siface_add(logger->siface, log);
    if (ret < 0) {
        vPortFree(log);
        return -1;
    }

    return 0;
}
#endif

/******************************************************************************/
#ifdef LOGGER
int logger_add_str(struct logger* logger,
                   const char* tag,
                   bool full,
                   const char* buf) {
    return logger_add(logger, tag, full, buf, strlen(buf));
}
#endif
