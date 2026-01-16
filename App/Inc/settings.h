#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include <stdbool.h>
#include "dev_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Settings storage size */
#define SETTINGS_SIZE     256

/* Flash layout - use entire flash for wear leveling */
#define SETTINGS_START_SECTOR     0


/* External declaration of default settings in flash */
extern const device_settings_t default_settings;

/* Functions */
bool settings_init(void);
bool settings_read(device_settings_t *settings);
bool settings_write(const device_settings_t *settings);

#ifdef __cplusplus
}
#endif

#endif /* SETTINGS_H */

