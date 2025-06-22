#ifndef SMARTRF_SETTINGS_H
#define SMARTRF_SETTINGS_H

#include <stdint.h>

// Define the registerSetting_t struct if not already defined
typedef struct {
    uint16_t addr;
    uint8_t  value;
} registerSetting_t;

// Extern declaration for the settings array
extern const registerSetting_t preferredSettings[];
extern const uint32_t preferredSettings_size;

#endif // SMARTRF_SETTINGS_H