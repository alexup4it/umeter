/*
 * String JSON
 */

#ifndef STRJSON_H_
#define STRJSON_H_

#include <stddef.h>

void strjson_init(char* json, size_t size);
int strjson_str(char* json, size_t size, const char* name, const char* value);
int strjson_int(char* json, size_t size, const char* name, int value);
int strjson_uint(char* json, size_t size, const char* name, unsigned int value);
int strjson_null(char* json, size_t size, const char* name);

#endif /* STRJSON_H_ */
