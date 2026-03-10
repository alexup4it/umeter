/*
 * String JSON
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024
 */

#include "strjson.h"

#include <stdlib.h>
#include <string.h>

/******************************************************************************/
void strjson_init(char* json) {
    strcpy(json, "{}");
}

// {"name": or ,"name":
static int add_name(char* json, const char* name) {
    size_t len = strlen(json);

    if (!len) {
        return -1;
    }

    json = &json[len - 1];

    if (len > 2) {
        strcpy(json, ",");
        json++;
    }

    strcpy(json, "\"");
    strcat(json, name);
    strcat(json, "\":");

    return 0;
}

/******************************************************************************/
void strjson_str(char* json, const char* name, const char* value) {
    if (add_name(json, name)) {
        return;
    }

    json = &json[strlen(json)];

    strcpy(json, "\"");
    strcat(json, value);
    strcat(json, "\"}");
}

/******************************************************************************/
void strjson_int(char* json, const char* name, int value) {
    if (add_name(json, name)) {
        return;
    }

    json = &json[strlen(json)];

    itoa(value, json, 10);
    strcat(json, "}");
}

/******************************************************************************/
void strjson_uint(char* json, const char* name, unsigned int value) {
    if (add_name(json, name)) {
        return;
    }

    json = &json[strlen(json)];

    utoa(value, json, 10);
    strcat(json, "}");
}

/******************************************************************************/
void strjson_null(char* json, const char* name) {
    if (add_name(json, name)) {
        return;
    }

    json = &json[strlen(json)];

    strcpy(json, "null}");
}
