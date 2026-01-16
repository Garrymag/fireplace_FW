#ifndef DEVICE_H
#define DEVICE_H

#include <stdint.h>
#include "dev_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Default settings stored in flash memory (placed in .flashmem section) */
const device_settings_t default_settings __attribute__((section(".rodata"))) = 
{

};




#ifdef __cplusplus
}
#endif

#endif // DEVICE_H
