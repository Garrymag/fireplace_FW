#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Settings storage size */
#define SETTINGS_SIZE     256

/* Flash layout - use entire flash for wear leveling */
#define SETTINGS_START_SECTOR     0

/* Default values for all settings */
#define DEFAULT_BRIGHTNESS        80
#define DEFAULT_VOLUME            50
#define DEFAULT_LANGUAGE          0
#define DEFAULT_TEMP_UNIT         0  /* 0 = Celsius, 1 = Fahrenheit */
#define DEFAULT_SCREEN_TIMEOUT    30

/* Main settings structure with sequence embedded */
typedef struct {
    uint32_t sequence;
    uint8_t brightness;
    uint8_t volume;
    uint8_t language;
    uint8_t temp_unit;
    uint16_t screen_timeout;
    uint8_t reserved[248];       /* Padding to make SETTINGS_SIZE bytes */
} device_settings_t;

/* Default settings stored in flash memory (PROGMEM) */
#define DEFAULT_SETTINGS \
{ \
    .sequence = 0, \
    .brightness = 80, \
    .volume = 50, \
    .language = 0, \
    .temp_unit = 0, \
    .screen_timeout = 30, \
    .reserved = {0}, \
}

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

