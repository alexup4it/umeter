/*
 * Logger
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <stdbool.h>
#include <stdint.h>

#include "siface.h"

#define LOGGER

/*---------------------------------------------------------------------------*/
/* Log levels                                                                */
/*---------------------------------------------------------------------------*/

enum log_level {
    LOG_INFO = 'I',
    LOG_WARN = 'W',
    LOG_ERR  = 'E',
};

/*---------------------------------------------------------------------------*/
/* Logger context                                                            */
/*---------------------------------------------------------------------------*/

struct logger {
    struct siface* siface;
};

/*---------------------------------------------------------------------------*/
/* USB active flag — set by task_logging after USB check                     */
/*---------------------------------------------------------------------------*/

extern volatile uint8_t log_usb_active;

/*---------------------------------------------------------------------------*/
/* Core API                                                                  */
/*---------------------------------------------------------------------------*/

void logger_init(struct logger* self, struct siface* siface);

#ifdef LOGGER
int logger_add(struct logger* self,
               enum log_level level,
               const char* tag,
               bool full,
               const char* buf,
               size_t len);
#else
inline static int logger_add(struct logger* logger,
                             enum log_level level,
                             const char* tag,
                             bool full,
                             const char* buf,
                             size_t len) {
    return 0;
}
#endif

#ifdef LOGGER
int logger_add_str(struct logger* self,
                   enum log_level level,
                   const char* tag,
                   bool full,
                   const char* buf);
#else
inline static int logger_add_str(struct logger* logger,
                                 enum log_level level,
                                 const char* tag,
                                 bool full,
                                 const char* buf) {
    return 0;
}
#endif

/*---------------------------------------------------------------------------*/
/* Convenience macros — check USB flag before any work                       */
/*---------------------------------------------------------------------------*/

#ifdef LOGGER

#    define LOG_I(logger, tag, msg)                                      \
        do {                                                             \
            if (log_usb_active)                                          \
                logger_add_str((logger), LOG_INFO, (tag), false, (msg)); \
        } while (0)

#    define LOG_W(logger, tag, msg)                                      \
        do {                                                             \
            if (log_usb_active)                                          \
                logger_add_str((logger), LOG_WARN, (tag), false, (msg)); \
        } while (0)

#    define LOG_E(logger, tag, msg)                                     \
        do {                                                            \
            if (log_usb_active)                                         \
                logger_add_str((logger), LOG_ERR, (tag), false, (msg)); \
        } while (0)

#    define LOG_I_BUF(logger, tag, buf, len)                                \
        do {                                                                \
            if (log_usb_active)                                             \
                logger_add((logger), LOG_INFO, (tag), false, (buf), (len)); \
        } while (0)

#    define LOG_W_BUF(logger, tag, buf, len)                                \
        do {                                                                \
            if (log_usb_active)                                             \
                logger_add((logger), LOG_WARN, (tag), false, (buf), (len)); \
        } while (0)

#    define LOG_E_BUF(logger, tag, buf, len)                               \
        do {                                                               \
            if (log_usb_active)                                            \
                logger_add((logger), LOG_ERR, (tag), false, (buf), (len)); \
        } while (0)

#    define LOG_I_FULL(logger, tag, msg)                                \
        do {                                                            \
            if (log_usb_active)                                         \
                logger_add_str((logger), LOG_INFO, (tag), true, (msg)); \
        } while (0)

#else

#    define LOG_I(logger, tag, msg) \
        do {                        \
        } while (0)
#    define LOG_W(logger, tag, msg) \
        do {                        \
        } while (0)
#    define LOG_E(logger, tag, msg) \
        do {                        \
        } while (0)
#    define LOG_I_BUF(logger, tag, buf, len) \
        do {                                 \
        } while (0)
#    define LOG_W_BUF(logger, tag, buf, len) \
        do {                                 \
        } while (0)
#    define LOG_E_BUF(logger, tag, buf, len) \
        do {                                 \
        } while (0)
#    define LOG_I_FULL(logger, tag, msg) \
        do {                             \
        } while (0)

#endif

#endif /* LOGGER_H_ */
