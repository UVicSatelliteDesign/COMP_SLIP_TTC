#include "main.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"
#include "CC1201_commands.h"
#include "CC1201_simple_link_reg_config.h"
#include "CC1201_reg.h" // Assuming CC1201_SendStrobe is defined here

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

// Read bit for register addresses
#define CC1201_READ_BIT                 0x80

HAL_StatusTypeDef CC1201_SoftReset(uint8_t *status_byte)
{
    printf("[DEBUG] Entered CC1201_SoftReset function\n\r");
    HAL_StatusTypeDef result = CC1201_SendStrobe(CC1201_STROBE_SOFT_RESET, status_byte);
    printf("[DEBUG] CC1201_SoftReset about to return: %d\n\r", result);
    return result;
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
    printf("[DEBUG] Entered CC1201_Nop function\n\r");
    HAL_StatusTypeDef result = CC1201_SendStrobe(CC1201_STROBE_NOP, status_byte);
    printf("[DEBUG] CC1201_Nop about to return: %d\n\r", result);
    return result;
}

/**
 * @brief Reads a single register from the CC1201 radio.
 *
 * This function is used to read the current value of a CC1201 configuration or status register.
 * For example, to read the `MARCSTATE` register, which indicates the current state of the radio
 * (e.g., IDLE, RX, TX, FSTXON, CALIBRATE, SLEEP), you would pass its defined address.
 * Note: The actual definition for `MARCSTATE` (e.g., `#define CC1201_MARCSTATE 0xXX`) should be
 * provided in a separate header file that lists all CC1201 register addresses.
 *
 * @param reg_addr The address of the register to read. The read bit (MSB) will be set internally.
 * @param read_data Pointer to a uint8_t where the read data will be stored.
 * @return HAL_StatusTypeDef Status of the SPI transmission (HAL_OK on success).
 */
HAL_StatusTypeDef CC1201_ReadStatus(uint16_t reg_addr, uint8_t *read_data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buffer[4];
    uint8_t rx_buffer[4];
    uint8_t buffer_size;

    if (reg_addr > 0xFF) {
        // Extended register single read: first byte is 0x2F with R/W bit set
        tx_buffer[0] = 0x2F | CC1201_READ_BIT; // Extended register READ command
        tx_buffer[1] = (uint8_t)(reg_addr & 0xFF); // Low byte of address
        tx_buffer[2] = 0x00; // Dummy byte for reading
        buffer_size = 3;
    } else {
        // Standard register
        tx_buffer[0] = (uint8_t)reg_addr | CC1201_READ_BIT;
        tx_buffer[1] = 0x00; // Dummy byte for reading
        buffer_size = 2;
    }

    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_RESET); // Pull CS low

    status = HAL_SPI_TransmitReceive(&CC1201_SPI_HANDLE, tx_buffer, rx_buffer, buffer_size, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_SET); // Pull CS high

    if (status == HAL_OK) {
        if (read_data != NULL) {
            if (reg_addr > 0xFF) {
                *read_data = rx_buffer[2]; // Extended register data
            } else {
                *read_data = rx_buffer[1]; // Standard register data
            }
        }
    }
    return status;
}

/**
 * @brief Writes a single byte to a register in the CC1201 radio.
 *
 * @param reg_addr The address of the register to write to.
 * @param write_data The byte of data to write to the register.
 * @return HAL_StatusTypeDef Status of the SPI transmission (HAL_OK on success).
 */
HAL_StatusTypeDef CC1201_WriteRegister(uint16_t reg_addr, uint8_t write_data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buffer[4];
    uint8_t buffer_size;

    if (reg_addr > 0xFF) {
        // Extended register single write: first byte is 0x2F with WRITE (no read bit)
        tx_buffer[0] = 0x2F; // Extended register WRITE command
        tx_buffer[1] = (uint8_t)(reg_addr & 0xFF); // Low byte of address
        tx_buffer[2] = write_data; // Data to write
        buffer_size = 3;
    } else {
        // Standard register
        tx_buffer[0] = (uint8_t)reg_addr; // Register address
        tx_buffer[1] = write_data; // Data to write
        buffer_size = 2;
    }

    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_RESET); // Pull CS low

    status = HAL_SPI_Transmit(&CC1201_SPI_HANDLE, tx_buffer, buffer_size, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_SET); // Pull CS high

    return status;
}

/**
 * @brief Reads the MARCSTATE register to get the current state of the radio.
 *
 * @param marc_state Pointer to a uint8_t where the MARCSTATE value will be stored.
 * @return HAL_StatusTypeDef Status of the SPI transmission (HAL_OK on success).
 */
HAL_StatusTypeDef CC1201_ReadMARCState(uint8_t *marc_state)
{
    return CC1201_ReadStatus(CC1201_MARCSTATE, marc_state);
}

/**
 * @brief Gets the number of bytes currently in the RX FIFO.
 *
 * @param num_bytes Pointer to a uint8_t where the number of RX bytes will be stored.
 * @return HAL_StatusTypeDef Status of the SPI transmission (HAL_OK on success).
 */
HAL_StatusTypeDef CC1201_GetNumRXBytes(uint8_t *num_bytes)
{
    return CC1201_ReadStatus(CC1201_NUM_RXBYTES, num_bytes);
}

/**
 * @brief Gets the number of bytes currently in the TX FIFO.
 *
 * @param num_bytes Pointer to a uint8_t where the number of TX bytes will be stored.
 * @return HAL_StatusTypeDef Status of the SPI transmission (HAL_OK on success).
 */
HAL_StatusTypeDef CC1201_GetNumTXBytes(uint8_t *num_bytes)
{
    return CC1201_ReadStatus(CC1201_NUM_TXBYTES, num_bytes);
}

/**
 * @brief Writes multiple bytes to the TX FIFO.
 *
 * @param data Pointer to the data array to be written to TX FIFO.
 * @param length Number of bytes to write.
 * @param status_byte Pointer to store the status byte returned.
 * @return HAL_StatusTypeDef Status of the SPI transmission (HAL_OK on success).
 */
HAL_StatusTypeDef CC1201_WriteTxFifo(uint8_t *data, uint8_t length, uint8_t *status_byte)
{
    if (data == NULL || status_byte == NULL || length == 0) {
        return HAL_ERROR;
    }
    
    HAL_StatusTypeDef status;
    uint8_t tx_buffer[2 + length]; // Command + address + data
    uint8_t rx_buffer[2 + length];
    
    // Build command: burst write to TX FIFO (0x3F | 0x40)
    tx_buffer[0] = 0x7F; // TX FIFO burst write
    
    // Copy data to transmit buffer
    for (uint8_t i = 0; i < length; i++) {
        tx_buffer[1 + i] = data[i];
    }
    
    // Pull CS low to start SPI transaction
    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_RESET);
    
    // Perform SPI transaction
    status = HAL_SPI_TransmitReceive(&hspi4, tx_buffer, rx_buffer, length + 1, HAL_MAX_DELAY);
    
    // Pull CS high to end SPI transaction
    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_SET);
    
    // Store status byte (first received byte)
    *status_byte = rx_buffer[0];
    
    return status;
}

/**
 * @brief Reads multiple bytes from the RX FIFO.
 *
 * @param data Pointer to the buffer where read data will be stored.
 * @param length Number of bytes to read.
 * @param status_byte Pointer to store the status byte returned.
 * @return HAL_StatusTypeDef Status of the SPI transmission (HAL_OK on success).
 */
HAL_StatusTypeDef CC1201_ReadRxFifo(uint8_t *data, uint8_t length, uint8_t *status_byte)
{
    if (data == NULL || status_byte == NULL || length == 0) {
        return HAL_ERROR;
    }
    
    HAL_StatusTypeDef status;
    uint8_t tx_buffer[1 + length]; // Command + dummy bytes
    uint8_t rx_buffer[1 + length];
    
    // Build command: burst read from RX FIFO (0x3F | 0x80 | 0x40)
    tx_buffer[0] = 0xFF; // RX FIFO burst read
    
    // Fill with dummy bytes for reading
    for (uint8_t i = 1; i <= length; i++) {
        tx_buffer[i] = 0x00;
    }
    
    // Pull CS low to start SPI transaction
    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_RESET);
    
    // Perform SPI transaction
    status = HAL_SPI_TransmitReceive(&hspi4, tx_buffer, rx_buffer, length + 1, HAL_MAX_DELAY);
    
    // Pull CS high to end SPI transaction
    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_SET);
    
    // Store status byte (first received byte)
    *status_byte = rx_buffer[0];
    
    // Copy received data
    for (uint8_t i = 0; i < length; i++) {
        data[i] = rx_buffer[1 + i];
    }
    
    return status;
}

/**
 * @brief Reads a single byte from the RX FIFO.
 *
 * @param data Pointer to store the single byte read from RX FIFO.
 * @param status_byte Pointer to store the status byte returned.
 * @return HAL_StatusTypeDef Status of the SPI transmission (HAL_OK on success).
 */
HAL_StatusTypeDef CC1201_ReadSingleRxFifo(uint8_t *data, uint8_t *status_byte)
{
    return CC1201_ReadRxFifo(data, 1, status_byte);
}

/**
 * @brief Writes a single byte to the TX FIFO.
 *
 * @param data Single byte to write to TX FIFO.
 * @param status_byte Pointer to store the status byte returned.
 * @return HAL_StatusTypeDef Status of the SPI transmission (HAL_OK on success).
 */
HAL_StatusTypeDef CC1201_WriteSingleTxFifo(uint8_t data, uint8_t *status_byte)
{
    return CC1201_WriteTxFifo(&data, 1, status_byte);
}
