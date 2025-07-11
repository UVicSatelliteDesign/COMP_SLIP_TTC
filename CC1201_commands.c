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

// Read bit for register addresses
#define CC1201_READ_BIT                 0x80

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
HAL_StatusTypeDef CC1201_ReadStatus(uint8_t reg_addr, uint8_t *read_data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buffer[2];
    uint8_t rx_buffer[2];

    // Set the read bit (MSB) for the register address
    tx_buffer[0] = reg_addr | CC1201_READ_BIT;
    tx_buffer[1] = 0x00; // Dummy byte for reading data

    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_RESET); // Pull CS low

    status = HAL_SPI_TransmitReceive(&CC1201_SPI_HANDLE, tx_buffer, rx_buffer, 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_SET); // Pull CS high

    if (status == HAL_OK) {
        if (read_data != NULL) {
            *read_data = rx_buffer[1]; // The actual data is in the second byte received
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
HAL_StatusTypeDef CC1201_WriteRegister(uint8_t reg_addr, uint8_t write_data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buffer[2];

    tx_buffer[0] = reg_addr; // Register address
    tx_buffer[1] = write_data; // Data to write

    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_RESET); // Pull CS low

    status = HAL_SPI_Transmit(&CC1201_SPI_HANDLE, tx_buffer, 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_SET); // Pull CS high

    return status;
}