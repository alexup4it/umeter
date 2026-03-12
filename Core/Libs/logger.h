/*
 * Logger
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2025
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <stdbool.h>

#include "siface.h"

#define LOGGER

struct logger {
    struct siface* siface;
};

#ifdef LOGGER
#endif

void logger_init(struct logger* logger, struct siface* siface);

#ifdef LOGGER
int logger_add(struct logger* logger,
               const char* tag,
               bool full,
               const char* buf,
               size_t len);
#else
inline static int logger_add(struct logger* logger,
                             const char* tag,
                             bool full,
                             const char* buf,
                             size_t len) {
    return 0;
}
#endif

#ifdef LOGGER
int logger_add_str(struct logger* logger,
                   const char* tag,
                   bool full,
                   const char* buf);
#else
inline static int logger_add_str(struct logger* logger,
                                 const char* tag,
                                 bool full,
                                 const char* buf) {
    return 0;
}
#endif

#endif /* LOGGER_H_ */
