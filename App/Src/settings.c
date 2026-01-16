#include "settings.h"
#include "w25qxx.h"
#include <string.h>

static uint32_t current_sequence = 0;
static uint32_t current_sector = SETTINGS_START_SECTOR;

bool settings_init(void) {
    // Find the most recent valid settings entry
    uint32_t max_sequence = 0;
    uint32_t max_sector = SETTINGS_START_SECTOR;

    for (uint32_t sector = SETTINGS_START_SECTOR; sector < SETTINGS_START_SECTOR + SETTINGS_SECTORS; sector++) {
        settings_entry_t entry;
        W25qxx_ReadBytes((uint8_t*)&entry, sector * w25qxx.SectorSize, sizeof(settings_entry_t));

        // Check if this sector has valid data (not all 0xFF)
        bool is_empty = true;
        for (uint32_t i = 0; i < sizeof(settings_entry_t); i++) {
            if (((uint8_t*)&entry)[i] != 0xFF) {
                is_empty = false;
                break;
            }
        }

        if (!is_empty) {
            device_settings_t temp_settings;
            memcpy(temp_settings.bytes, entry.data, SETTINGS_SIZE);
            if (temp_settings.fields.sequence > max_sequence) {
                max_sequence = temp_settings.fields.sequence;
                max_sector = sector;
            }
        }
    }

    current_sequence = max_sequence;
    current_sector = max_sector;

    return true;
}

bool settings_read(device_settings_t *settings) {
    if (settings == NULL) return false;

    settings_entry_t entry;
    W25qxx_ReadBytes((uint8_t*)&entry, current_sector * w25qxx.SectorSize, sizeof(settings_entry_t));

    // Check if data is valid
    bool is_empty = true;
    for (uint32_t i = 0; i < sizeof(settings_entry_t); i++) {
        if (((uint8_t*)&entry)[i] != 0xFF) {
            is_empty = false;
            break;
        }
    }

    if (is_empty) {
        // No valid settings, return default (all zeros)
        memset(settings, 0, sizeof(device_settings_t));
    } else {
        memcpy(settings->bytes, entry.data, SETTINGS_SIZE);
    }

    return true;
}

bool settings_write(const device_settings_t *settings) {
    if (settings == NULL) return false;

    // Increment sequence number
    current_sequence++;

    // Find next sector for wear leveling
    current_sector++;
    if (current_sector >= SETTINGS_START_SECTOR + SETTINGS_SECTORS) {
        current_sector = SETTINGS_START_SECTOR;
    }

    // Erase the sector
    W25qxx_EraseSector(current_sector);

    // Prepare entry with sequence included in data
    settings_entry_t entry;
    device_settings_t temp_settings = *settings;
    temp_settings.fields.sequence = current_sequence;
    memcpy(entry.data, temp_settings.bytes, SETTINGS_SIZE);

    // Write to flash
    W25qxx_WriteSector((uint8_t*)&entry, current_sector, 0, sizeof(settings_entry_t));

    return true;
}
