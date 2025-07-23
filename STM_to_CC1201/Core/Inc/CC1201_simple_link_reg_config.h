#ifndef CC1201_SIMPLE_LINK_REG_CONFIG_H
#define CC1201_SIMPLE_LINK_REG_CONFIG_H

#include "main.h"

// Structure for register settings
typedef struct {
    uint16_t addr;
    uint8_t data;
} registerSetting_t;

// Function prototypes
HAL_StatusTypeDef CC1201_SendStrobe(uint8_t strobe_command, uint8_t *status_byte);
HAL_StatusTypeDef CC1201_WriteRegisterConfig(const registerSetting_t *settings, uint16_t num_settings);
uint16_t CC1201_GetNumPreferredSettings(void);
const registerSetting_t* CC1201_GetPreferredSettings(void);

#endif // CC1201_SIMPLE_LINK_REG_CONFIG_H
