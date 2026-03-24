/*
 * String JSON
 *
 * Every mutating function checks the remaining buffer space before writing.
 * On overflow the JSON string stays unchanged and the function returns -1.
 */

#include "strjson.h"

#include <stdlib.h>
#include <string.h>

/* Copy src into dst, return number of chars written.  No NUL terminator. */
static size_t scat(char* dst, const char* src) {
    const char* s = src;
    while (*s) {
        *dst++ = *s++;
    }
    return (size_t)(s - src);
}

/*
 * Find trailing '}', return its offset and whether a comma is needed.
 * Returns -1 on malformed input.
 */
static int field_start(const char* json, size_t* pos, int* need_comma) {
    size_t len = strlen(json);

    if (len == 0 || json[len - 1] != '}') {
        return -1;
    }

    *pos        = len - 1;
    *need_comma = (len > 2) ? 1 : 0;
    return 0;
}

/* Required bytes for [,]"name":VALUE} */
static size_t field_len(int comma, const char* name, size_t value_len) {
    return (size_t)comma + 1 + strlen(name) + 1 + 1 + value_len + 1;
}

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

/******************************************************************************/
int strjson_str(char* json, size_t size, const char* name, const char* value) {
    size_t pos;
    int comma;

    if (field_start(json, &pos, &comma)) {
        return -1;
    }

    size_t vlen   = strlen(value);
    size_t needed = field_len(comma, name, vlen + 2);

    if (pos + needed >= size) {
        return -1;
    }

    char* p = json + pos;
    if (comma) {
        p += scat(p, ",");
    }
    p += scat(p, "\"");
    p += scat(p, name);
    p += scat(p, "\":\"");
    p += scat(p, value);
    p += scat(p, "\"}");
    *p = '\0';

    return 0;
}

/******************************************************************************/
int strjson_int(char* json, size_t size, const char* name, int value) {
    size_t pos;
    int comma;

    if (field_start(json, &pos, &comma)) {
        return -1;
    }

    char numbuf[12];
    itoa(value, numbuf, 10);
    size_t nlen   = strlen(numbuf);
    size_t needed = field_len(comma, name, nlen);

    if (pos + needed >= size) {
        return -1;
    }

    char* p = json + pos;
    if (comma) {
        p += scat(p, ",");
    }
    p += scat(p, "\"");
    p += scat(p, name);
    p += scat(p, "\":");
    p += scat(p, numbuf);
    p += scat(p, "}");
    *p = '\0';

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

    char numbuf[11];
    utoa(value, numbuf, 10);
    size_t nlen   = strlen(numbuf);
    size_t needed = field_len(comma, name, nlen);

    if (pos + needed >= size) {
        return -1;
    }

    char* p = json + pos;
    if (comma) {
        p += scat(p, ",");
    }
    p += scat(p, "\"");
    p += scat(p, name);
    p += scat(p, "\":");
    p += scat(p, numbuf);
    p += scat(p, "}");
    *p = '\0';

    return 0;
}

/******************************************************************************/
int strjson_null(char* json, size_t size, const char* name) {
    size_t pos;
    int comma;

    if (field_start(json, &pos, &comma)) {
        return -1;
    }

    size_t needed = field_len(comma, name, 4);

    if (pos + needed >= size) {
        return -1;
    }

    char* p = json + pos;
    if (comma) {
        p += scat(p, ",");
    }
    p += scat(p, "\"");
    p += scat(p, name);
    p += scat(p, "\":null}");
    *p = '\0';

    return 0;
}
