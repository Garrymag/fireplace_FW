#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SETTINGS_SIZE 256
#define SETTINGS_SECTORS 4
#define SETTINGS_START_SECTOR 1  // Avoid sector 0 used by main memory

typedef struct {
    uint8_t data[SETTINGS_SIZE];
} settings_entry_t;

typedef union {
    uint8_t bytes[SETTINGS_SIZE];
    struct {
        uint32_t sequence;
        uint8_t settings_data[SETTINGS_SIZE - sizeof(uint32_t)];
    } fields;
    // Add more fields here later
} device_settings_t;

// Functions
bool settings_init(void);
bool settings_read(device_settings_t *settings);
bool settings_write(const device_settings_t *settings);

#ifdef __cplusplus
}
#endif

#endif // SETTINGS_H
