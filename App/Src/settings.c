#include "settings.h"
#include "w25qxx.h"
#include <string.h>

/* Static state */
static uint32_t current_sequence = 0;
static uint32_t current_sector = SETTINGS_START_SECTOR;
static bool initialized = false;



/* Initialize settings to default values from flash */
static void settings_load_defaults(device_settings_t *settings)
{
    if (settings == NULL) return;
    memcpy(settings, &default_settings, sizeof(device_settings_t));
}

bool settings_init(void)
{
    if (initialized) {
        return true;
    }
    
    /* Initialize W25QXX flash driver */
    if (!W25qxx_Init()) {
        return false;
    }
    
    uint32_t max_sequence = 0;
    uint32_t found_sector = SETTINGS_START_SECTOR;
    bool found_any = false;
    device_settings_t settings;
    
    /* Scan all sectors to find the most recent valid settings */
    for (uint32_t sector = 0; sector < w25qxx.SectorCount; sector++) {
        uint32_t sector_addr = sector * w25qxx.SectorSize;
        W25qxx_ReadBytes((uint8_t*)&settings, sector_addr, sizeof(device_settings_t));
        
        /* Check if this sector has valid data (not all 0xFF) */
        bool is_empty = true;
        for (uint32_t i = 0; i < sizeof(device_settings_t); i++) {
            if (((uint8_t*)&settings)[i] != 0xFF) {
                is_empty = false;
                break;
            }
        }
        
        if (!is_empty) {
            found_any = true;
            if (settings.sequence > max_sequence) {
                max_sequence = settings.sequence;
                found_sector = sector;
            }
        }
    }
    
    if (found_any) {
        current_sequence = max_sequence;
        current_sector = found_sector;
    } else {
        /* No valid settings found, use defaults */
        current_sequence = 0;
        current_sector = SETTINGS_START_SECTOR;
    }
    
    initialized = true;
    return true;
}

bool settings_read(device_settings_t *settings)
{
    if (settings == NULL) return false;
    
    if (!initialized) {
        if (!settings_init()) {
            return false;
        }
    }
    
    /* Find sector with maximum sequence */
    uint32_t max_sequence = 0;
    uint32_t best_sector = SETTINGS_START_SECTOR;
    device_settings_t temp;
    bool found_any = false;
    
    for (uint32_t sector = 0; sector < w25qxx.SectorCount; sector++) {
        uint32_t sector_addr = sector * w25qxx.SectorSize;
        W25qxx_ReadBytes((uint8_t*)&temp, sector_addr, sizeof(device_settings_t));
        
        /* Check if sector has valid data */
        bool is_empty = true;
        for (uint32_t i = 0; i < sizeof(device_settings_t); i++) {
            if (((uint8_t*)&temp)[i] != 0xFF) {
                is_empty = false;
                break;
            }
        }
        
        if (!is_empty) {
            found_any = true;
            if (temp.sequence > max_sequence) {
                max_sequence = temp.sequence;
                best_sector = sector;
            }
        }
    }
    
    if (found_any) {
        /* Read from best sector */
        uint32_t sector_addr = best_sector * w25qxx.SectorSize;
        W25qxx_ReadBytes((uint8_t*)settings, sector_addr, sizeof(device_settings_t));
    } else {
        /* No valid settings in flash, copy from default settings in flash memory */
        settings_load_defaults(settings);
    }
    
    return true;
}

bool settings_write(const device_settings_t *settings)
{
    if (settings == NULL) return false;
    
    if (!initialized) {
        settings_init();
    }
    
    /* Move to next sector (simple round-robin) */
    current_sector++;
    if (current_sector >= w25qxx.SectorCount) {
        current_sector = SETTINGS_START_SECTOR;
    }
    
    /* Increment sequence number */
    current_sequence++;
    
    /* Prepare data with updated sequence */
    device_settings_t data = *settings;
    data.sequence = current_sequence;
    
    /* Erase sector */
    W25qxx_EraseSector(current_sector);
    
    /* Write to flash */
    uint32_t sector_addr = current_sector * w25qxx.SectorSize;
    W25qxx_WriteSector((uint8_t*)&data, sector_addr, 0, sizeof(device_settings_t));
    
    return true;
}

