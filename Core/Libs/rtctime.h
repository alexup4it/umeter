#ifndef RTCTIME_H
#define RTCTIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Global Unix timestamp, updated by rtctime_update(). */
extern uint32_t timestamp;

/**
 * Convert calendar date/time to Unix epoch (seconds since 1970-01-01 00:00:00).
 */
uint32_t date_to_epoch(int year, int month, int day, int h, int m, int s);

/**
 * Convert Unix epoch to calendar date/time components.
 */
void epoch_to_date(uint32_t epoch,
                   int* year,
                   int* month,
                   int* day,
                   int* h,
                   int* m,
                   int* s);

/**
 * Read current RTC calendar and return Unix timestamp.
 */
uint32_t get_timestamp(void);

/**
 * Update global timestamp from RTC.
 */
void rtctime_read(void);

/**
 * Set RTC calendar from Unix timestamp and update global `timestamp` variable.
 */
void set_timestamp(uint32_t unix_ts);

#ifdef __cplusplus
}
#endif

#endif /* RTCTIME_H */
