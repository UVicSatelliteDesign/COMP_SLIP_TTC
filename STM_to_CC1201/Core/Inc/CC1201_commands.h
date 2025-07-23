#ifndef CC1201_COMMANDS_H
#define CC1201_COMMANDS_H

#include <stdint.h>
#include "stm32h7xx_hal.h" // For HAL_StatusTypeDef

// Function prototypes for CC1201 strobe commands
HAL_StatusTypeDef CC1201_SoftReset(uint8_t *status_byte);
HAL_StatusTypeDef CC1201_FastTxOn(uint8_t *status_byte);
HAL_StatusTypeDef CC1201_OscOff(uint8_t *status_byte);
HAL_StatusTypeDef CC1201_CalFreqSynth(uint8_t *status_byte);
HAL_StatusTypeDef CC1201_EnterRxMode(uint8_t *status_byte);
HAL_StatusTypeDef CC1201_EnterTxMode(uint8_t *status_byte);
HAL_StatusTypeDef CC1201_EnterIdleMode(uint8_t *status_byte);
HAL_StatusTypeDef CC1201_AutoFreqComp(uint8_t *status_byte);
HAL_StatusTypeDef CC1201_WakeOnRadio(uint8_t *status_byte);
HAL_StatusTypeDef CC1201_EnterSleepMode(uint8_t *status_byte);
HAL_StatusTypeDef CC1201_FlushRxFifo(uint8_t *status_byte);
HAL_StatusTypeDef CC1201_FlushTxFifo(uint8_t *status_byte);
HAL_StatusTypeDef CC1201_WorReset(uint8_t *status_byte);
HAL_StatusTypeDef CC1201_Nop(uint8_t *status_byte);

// Function prototype for reading a status register
HAL_StatusTypeDef CC1201_ReadStatus(uint16_t reg_addr, uint8_t *read_data);

// Function prototype for writing to a register
HAL_StatusTypeDef CC1201_WriteRegister(uint16_t reg_addr, uint8_t write_data);

// Function prototypes for reading status and FIFO bytes
HAL_StatusTypeDef CC1201_ReadMARCState(uint8_t *marc_state);
HAL_StatusTypeDef CC1201_GetNumRXBytes(uint8_t *num_bytes);
HAL_StatusTypeDef CC1201_GetNumTXBytes(uint8_t *num_bytes);

#endif // CC1201_COMMANDS_H
