/*
 * String JSON
 *
 * Every mutating function checks the remaining buffer space before writing.
 * On overflow the JSON string stays unchanged and the function returns -1.
 */

#include "strjson.h"

#include <stdio.h>
#include <string.h>

/******************************************************************************/
void strjson_init(char* json, size_t size) {
    if (size < 3) {
        if (size > 0) {
            json[0] = '\0';
        }
        return;
    }
    json[0] = '{';
    json[1] = '}';
    json[2] = '\0';
}

/*
 * Prepare a write position: find the trailing '}', return its offset and
 * whether a separating comma is needed.  Returns -1 when the buffer is in
 * an unexpected state.
 */
static int field_start(const char* json, size_t* pos, int* need_comma) {
    size_t len = strlen(json);

    if (len == 0 || json[len - 1] != '}') {
        return -1;
    }

    *pos        = len - 1; /* position of the closing '}' */
    *need_comma = (len > 2) ? 1 : 0;
    return 0;
}

/******************************************************************************/
int strjson_str(char* json, size_t size, const char* name, const char* value) {
    size_t pos;
    int comma;

    if (field_start(json, &pos, &comma)) {
        return -1;
    }

    size_t rem  = size - pos;
    int written = snprintf(json + pos,
                           rem,
                           "%s\"%s\":\"%s\"}",
                           comma ? "," : "",
                           name,
                           value);

    if (written < 0 || (size_t)written >= rem) {
        /* Truncated — restore closing brace */
        json[pos]     = '}';
        json[pos + 1] = '\0';
        return -1;
    }

    return 0;
}

/******************************************************************************/
int strjson_int(char* json, size_t size, const char* name, int value) {
    size_t pos;
    int comma;

    if (field_start(json, &pos, &comma)) {
        return -1;
    }

    size_t rem  = size - pos;
    int written = snprintf(json + pos,
                           rem,
                           "%s\"%s\":%d}",
                           comma ? "," : "",
                           name,
                           value);

    if (written < 0 || (size_t)written >= rem) {
        json[pos]     = '}';
        json[pos + 1] = '\0';
        return -1;
    }

    return 0;
}

/******************************************************************************/
int strjson_uint(char* json,
                 size_t size,
                 const char* name,
                 unsigned int value) {
    size_t pos;
    int comma;

    if (field_start(json, &pos, &comma)) {
        return -1;
    }

    size_t rem  = size - pos;
    int written = snprintf(json + pos,
                           rem,
                           "%s\"%s\":%u}",
                           comma ? "," : "",
                           name,
                           value);

    if (written < 0 || (size_t)written >= rem) {
        json[pos]     = '}';
        json[pos + 1] = '\0';
        return -1;
    }

    return 0;
}

/******************************************************************************/
int strjson_null(char* json, size_t size, const char* name) {
    size_t pos;
    int comma;

    if (field_start(json, &pos, &comma)) {
        return -1;
    }

    size_t rem = size - pos;
    int written =
        snprintf(json + pos, rem, "%s\"%s\":null}", comma ? "," : "", name);

    if (written < 0 || (size_t)written >= rem) {
        json[pos]     = '}';
        json[pos + 1] = '\0';
        return -1;
    }

    return 0;
}
