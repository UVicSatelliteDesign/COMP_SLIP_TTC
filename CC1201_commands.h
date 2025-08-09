#ifndef CC1201_COMMANDS_H
#define CC1201_COMMANDS_H

#include <stdint.h>
#include "stm32h7xx_hal.h" // For HAL_StatusTypeDef

// Define the SPI handle to be used for CC1201 communication
extern SPI_HandleTypeDef hspi4; // Using SPI4
#define CC1201_SPI_HANDLE hspi4

// Define the Chip Select (CS) pin for the CC1201
#define CC1201_CS_PORT                  GPIOE
#define CC1201_CS_PIN                   GPIO_PIN_11

// Define the GPIO interrupt pin for RX/TX indications
#define CC1201_INTERRUPT_PORT           GPIOD
#define CC1201_INTERRUPT_PIN            GPIO_PIN_5
#define CC1201_INTERRUPT_EXTI_IRQn      EXTI9_5_IRQn

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

// Function prototypes for FIFO operations
HAL_StatusTypeDef CC1201_WriteTxFifo(uint8_t *data, uint8_t length, uint8_t *status_byte);
HAL_StatusTypeDef CC1201_ReadRxFifo(uint8_t *data, uint8_t length, uint8_t *status_byte);
HAL_StatusTypeDef CC1201_ReadSingleRxFifo(uint8_t *data, uint8_t *status_byte);
HAL_StatusTypeDef CC1201_WriteSingleTxFifo(uint8_t data, uint8_t *status_byte);

#endif // CC1201_COMMANDS_H
