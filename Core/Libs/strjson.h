/*
 * String JSON
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024
 */

#ifndef STRJSON_H_
#define STRJSON_H_

void strjson_init(char* json);
void strjson_str(char* json, const char* name, const char* value);
void strjson_int(char* json, const char* name, int value);
void strjson_uint(char* json, const char* name, unsigned int value);
void strjson_null(char* json, const char* name);

#endif /* STRJSON_H_ */
