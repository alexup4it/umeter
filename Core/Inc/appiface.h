/*
 * Application interface
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024
 */

#ifndef APPIFACE_H_
#define APPIFACE_H_

#include "params.h"

struct appiface {
    params_t uparams;
};

extern struct appiface appif;

int appiface(void* iface);

#endif /* APPIFACE_H_ */
