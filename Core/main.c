#include "Drivers/cc1201.h"
#include "Drivers/smartrf_settings.h"

int main(void)
{
    // ...existing STM32 initialization code...

    // Example: Iterate through preferredSettings and write to CC1201
    for (uint32_t i = 0; i < preferredSettings_size; i++) {
        uint16_t reg_addr = preferredSettings[i].addr;
        uint8_t reg_value = preferredSettings[i].value;
        // TODO: Call your SPI register write function here, e.g.:
        // cc1201_write_register(reg_addr, reg_value);
    }

    // ...rest of your application...
}