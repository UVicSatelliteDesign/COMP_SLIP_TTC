#include "CC1201_commands.h"
#include "CC1201_simple_link_reg_config.h" // Assuming CC1201_SendStrobe is defined here

// Strobe command values for CC1201 (from datasheet)
#define CC1201_STROBE_SOFT_RESET        0x30
#define CC1201_STROBE_FAST_TX_ON        0x31
#define CC1201_STROBE_OSC_OFF           0x32
#define CC1201_STROBE_CAL_FREQ_SYNTH    0x33
#define CC1201_STROBE_RX                0x34
#define CC1201_STROBE_TX                0x35
#define CC1201_STROBE_IDLE              0x36
#define CC1201_STROBE_AUTO_FREQ_COMP    0x37
#define CC1201_STROBE_WAKE_ON_RADIO     0x38
#define CC1201_STROBE_SLEEP             0x39
#define CC1201_STROBE_FLUSH_RX          0x3A
#define CC1201_STROBE_FLUSH_TX          0x3B
#define CC1201_STROBE_WOR_RESET         0x3C
#define CC1201_STROBE_NOP               0x3D

HAL_StatusTypeDef CC1201_SoftReset(uint8_t *status_byte)
{
    return CC1201_SendStrobe(CC1201_STROBE_SOFT_RESET, status_byte);
}

HAL_StatusTypeDef CC1201_FastTxOn(uint8_t *status_byte)
{
    return CC1201_SendStrobe(CC1201_STROBE_FAST_TX_ON, status_byte);
}

HAL_StatusTypeDef CC1201_OscOff(uint8_t *status_byte)
{
    return CC1201_SendStrobe(CC1201_STROBE_OSC_OFF, status_byte);
}

HAL_StatusTypeDef CC1201_CalFreqSynth(uint8_t *status_byte)
{
    return CC1201_SendStrobe(CC1201_STROBE_CAL_FREQ_SYNTH, status_byte);
}

HAL_StatusTypeDef CC1201_EnterRxMode(uint8_t *status_byte)
{
    return CC1201_SendStrobe(CC1201_STROBE_RX, status_byte);
}

HAL_StatusTypeDef CC1201_EnterTxMode(uint8_t *status_byte)
{
    return CC1201_SendStrobe(CC1201_STROBE_TX, status_byte);
}

HAL_StatusTypeDef CC1201_EnterIdleMode(uint8_t *status_byte)
{
    return CC1201_SendStrobe(CC1201_STROBE_IDLE, status_byte);
}

HAL_StatusTypeDef CC1201_AutoFreqComp(uint8_t *status_byte)
{
    return CC1201_SendStrobe(CC1201_STROBE_AUTO_FREQ_COMP, status_byte);
}

HAL_StatusTypeDef CC1201_WakeOnRadio(uint8_t *status_byte)
{
    return CC1201_SendStrobe(CC1201_STROBE_WAKE_ON_RADIO, status_byte);
}

HAL_StatusTypeDef CC1201_EnterSleepMode(uint8_t *status_byte)
{
    return CC1201_SendStrobe(CC1201_STROBE_SLEEP, status_byte);
}

HAL_StatusTypeDef CC1201_FlushRxFifo(uint8_t *status_byte)
{
    return CC1201_SendStrobe(CC1201_STROBE_FLUSH_RX, status_byte);
}

HAL_StatusTypeDef CC1201_FlushTxFifo(uint8_t *status_byte)
{
    return CC1201_SendStrobe(CC1201_STROBE_FLUSH_TX, status_byte);
}

HAL_StatusTypeDef CC1201_WorReset(uint8_t *status_byte)
{
    return CC1201_SendStrobe(CC1201_STROBE_WOR_RESET, status_byte);
}

HAL_StatusTypeDef CC1201_Nop(uint8_t *status_byte)
{
    return CC1201_SendStrobe(CC1201_STROBE_NOP, status_byte);
}