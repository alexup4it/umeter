#include "rtctime.h"

#include "rtc.h"
#include "stm32f4xx_hal.h"

/* Global timestamp maintained by task_default / set_timestamp */
extern volatile uint32_t timestamp;

/*---------------------------------------------------------------------------*/
/* Internal helpers                                                          */
/*---------------------------------------------------------------------------*/

static int is_leap_year(int y) {
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

static const uint8_t mdays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/*---------------------------------------------------------------------------*/
/* Public API                                                                */
/*---------------------------------------------------------------------------*/

uint32_t date_to_epoch(int year, int month, int day, int h, int m, int s) {
    uint32_t days = 0;
    for (int y = 1970; y < year; y++) {
        days += is_leap_year(y) ? 366 : 365;
    }
    for (int mo = 1; mo < month; mo++) {
        days += mdays[mo - 1];
        if (mo == 2 && is_leap_year(year)) {
            days++;
        }
    }
    days += day - 1;
    return days * 86400U + (uint32_t)h * 3600U + (uint32_t)m * 60U + s;
}

void epoch_to_date(uint32_t epoch,
                   int* year,
                   int* month,
                   int* day,
                   int* h,
                   int* m,
                   int* s) {
    uint32_t rem  = epoch % 86400U;
    uint32_t days = epoch / 86400U;
    *h            = rem / 3600;
    rem %= 3600;
    *m = rem / 60;
    *s = rem % 60;

    int y = 1970;
    for (;;) {
        int yd = is_leap_year(y) ? 366 : 365;
        if (days < (uint32_t)yd) {
            break;
        }
        days -= yd;
        y++;
    }
    *year = y;

    int mo = 1;
    while (mo <= 12) {
        int md = mdays[mo - 1];
        if (mo == 2 && is_leap_year(y)) {
            md++;
        }
        if (days < (uint32_t)md) {
            break;
        }
        days -= md;
        mo++;
    }
    *month = mo;
    *day   = days + 1;
}

uint32_t get_timestamp(void) {
    RTC_TimeTypeDef t;
    RTC_DateTypeDef d;
    HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);
    return date_to_epoch(2000 + d.Year,
                         d.Month,
                         d.Date,
                         t.Hours,
                         t.Minutes,
                         t.Seconds);
}

void set_timestamp(uint32_t unix_ts) {
    int year, month, day, h, m, s;
    epoch_to_date(unix_ts, &year, &month, &day, &h, &m, &s);

    RTC_TimeTypeDef sTime = {0};
    sTime.Hours           = h;
    sTime.Minutes         = m;
    sTime.Seconds         = s;
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    RTC_DateTypeDef sDate = {0};
    sDate.Year            = year - 2000;
    sDate.Month           = month;
    sDate.Date            = day;
    sDate.WeekDay         = RTC_WEEKDAY_MONDAY; /* unused */
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    /* Update the global timestamp variable */
    timestamp = unix_ts;
}
