/*
 * Logger
 */

#include "logger.h"

#include <stdlib.h>
#include <string.h>

#include "cmsis_os.h"
#include "queue.h"

#ifdef LOGGER
#endif

#define MAX_DATA_LEN 96

volatile uint8_t log_usb_active = 0;

/******************************************************************************/
void logger_init(struct logger* logger, struct siface* siface) {
    memset(logger, 0, sizeof(*logger));
    logger->siface = siface;
}

/******************************************************************************/
/**
 * Estimate escaped length for a buffer: each \r, \n, \ becomes 2 chars.
 */
static size_t escaped_len(const char* buf, size_t len) {
    size_t extra = 0;

    for (size_t i = 0; i < len; i++) {
        if (buf[i] == '\r' || buf[i] == '\n' || buf[i] == '\\') {
            extra++;
        }
    }
    return len + extra;
}

/**
 * Copy buf into dst, escaping \r → backslash+r, \n → backslash+n,
 * \ → backslash+backslash.  Non-printable chars replaced with '*'.
 * Returns number of bytes written.
 */
static size_t escape_copy(char* dst, const char* buf, size_t len) {
    size_t w = 0;

    for (size_t i = 0; i < len; i++) {
        if (buf[i] == '\r') {
            dst[w++] = '\\';
            dst[w++] = 'r';
        } else if (buf[i] == '\n') {
            dst[w++] = '\\';
            dst[w++] = 'n';
        } else if (buf[i] == '\\') {
            dst[w++] = '\\';
            dst[w++] = '\\';
        } else if ((buf[i] < 0x20) || (buf[i] > 0x7E)) {
            dst[w++] = '*';
        } else {
            dst[w++] = buf[i];
        }
    }
    return w;
}

/******************************************************************************/
#ifdef LOGGER
int logger_add(struct logger* logger,
               enum log_level level,
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

    /* Format: "L/TAG,ticks,<escaped_msg>\r\n\0"
     *   L = level char (1)
     *   / = separator  (1)
     *   TAG
     *   , = separator  (1)
     *   ticks
     *   , = separator  (1)
     *   escaped message
     *   \r\n\0          (3)
     */
    size_t esc_len = escaped_len(buf, len);
    ll = 1 + 1 + strlen(tag) + 1 + strlen(ticks) + 1 + esc_len + 2 + 1;

    log = pvPortMalloc(ll);
    if (!log) {
        return -1;
    }

    /* Level prefix */
    log[0] = (char)level;
    log[1] = '/';
    log[2] = '\0';

    strcat(log, tag);
    strcat(log, ",");
    strcat(log, ticks);
    strcat(log, ",");

    /* Escaped message body */
    size_t hdr_len = strlen(log);
    size_t written = escape_copy(&log[hdr_len], buf, len);

    /* Line terminator */
    log[hdr_len + written]     = '\r';
    log[hdr_len + written + 1] = '\n';
    log[hdr_len + written + 2] = '\0';

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
                   enum log_level level,
                   const char* tag,
                   bool full,
                   const char* buf) {
    return logger_add(logger, level, tag, full, buf, strlen(buf));
}
#endif
