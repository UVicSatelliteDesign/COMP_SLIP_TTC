#include "main.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"
#include "CC1201_reg.h" // Include main.h to access SPI handle and CS pin definitions

static const registerSetting_t preferredSettings[]=
{
  {CC1201_IOCFG2,            0x07},
  {CC1201_SYNC3,             0x55},
  {CC1201_SYNC2,             0x55},
  {CC1201_SYNC1,             0x7A},
  {CC1201_SYNC0,             0x0E},
  {CC1201_SYNC_CFG1,         0x48},
  {CC1201_SYNC_CFG0,         0x23},
  {CC1201_DEVIATION_M,       0x47},
  {CC1201_MODCFG_DEV_E,      0x0B},
  {CC1201_DCFILT_CFG,        0x56},
  {CC1201_PREAMBLE_CFG0,     0xBA},
  {CC1201_IQIC,              0xC8},
  {CC1201_CHAN_BW,           0x84},
  {CC1201_MDMCFG1,           0x40},
  {CC1201_MDMCFG0,           0x05},
  {CC1201_SYMBOL_RATE2,      0x94},
  {CC1201_SYMBOL_RATE1,      0x7A},
  {CC1201_SYMBOL_RATE0,      0xE1},
  {CC1201_AGC_REF,           0x3E},
  {CC1201_AGC_CS_THR,        0xF1},
  {CC1201_AGC_CFG1,          0x11},
  {CC1201_AGC_CFG0,          0x90},
  {CC1201_FS_CFG,            0x12},
  {CC1201_PKT_CFG2,          0x00},
  {CC1201_PKT_CFG0,          0x20},
  {CC1201_PKT_LEN,           0xFF},
  {CC1201_IF_MIX_CFG,        0x18},
  {CC1201_TOC_CFG,           0x03},
  {CC1201_MDMCFG2,           0x02},
  {CC1201_FREQ2,             0x56},
  {CC1201_FREQ1,             0xCC},
  {CC1201_FREQ0,             0xCC},
  {CC1201_IF_ADC1,           0xEE},
  {CC1201_IF_ADC0,           0x10},
  {CC1201_FS_DIG1,           0x07},
  {CC1201_FS_DIG0,           0xAA},
  {CC1201_FS_CAL1,           0x40},
  {CC1201_FS_CAL0,           0x0E},
  {CC1201_FS_DIVTWO,         0x03},
  {CC1201_FS_DSM0,           0x33},
  {CC1201_FS_DVC0,           0x17},
  {CC1201_FS_PFD,            0x00},
  {CC1201_FS_PRE,            0x6E},
  {CC1201_FS_REG_DIV_CML,    0x1C},
  {CC1201_FS_SPARE,          0xAC},
  {CC1201_FS_VCO0,           0xB5},
  {CC1201_IFAMP,             0x05},
  {CC1201_XOSC5,             0x0E},
  {CC1201_XOSC1,             0x03},
};

/**
 * @brief Sends a strobe command to the CC1201 radio.
 * Strobe commands are used to control the state of the CC1201, such as changing power states (e.g., idle, RX, TX),
 * calibrating the frequency synthesizer, or flushing FIFOs. A comprehensive list of strobe commands
 * can be found in the CC1201 datasheet.
 *
 * @param strobe_command The specific strobe command byte to send.
 * @param status_byte Pointer to a uint8_t where the status byte received from the CC1201 will be stored.
 *                    Can be NULL if the status byte is not needed.
 * @return HAL_StatusTypeDef Status of the SPI transmission (HAL_OK on success).
 */
HAL_StatusTypeDef CC1201_SendStrobe(uint8_t strobe_command, uint8_t *status_byte)
{
    HAL_StatusTypeDef status;
    uint8_t rx_data;

    // Early readiness check to avoid blocking if SPI not initialized yet
    HAL_SPI_StateTypeDef spi_state = HAL_SPI_GetState(&CC1201_SPI_HANDLE);
    printf("[DEBUG] Enter CC1201_SendStrobe cmd=0x%02X, SPI state=%d\n\r", strobe_command, (int)spi_state);
    if (spi_state != HAL_SPI_STATE_READY) {
        printf("[DEBUG] SPI not ready, skipping strobe\n\r");
        if (status_byte) { *status_byte = 0x00; }
        return HAL_ERROR;
    }

    // Ensure SPI is ready and CS is toggled with brief gaps
    printf("[DEBUG] SPI state before CS low: %d\n\r", (int)HAL_SPI_GetState(&CC1201_SPI_HANDLE));
    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_RESET); // Pull CS low
    
    // Small CS setup delay without relying on SysTick (avoid HAL_Delay hang if tick not running)
    volatile uint32_t i;
    for (i = 0; i < 300; ++i) {
        __NOP();
    }

    // Perform SPI transaction
    printf("[DEBUG] About to SPI strobe 0x%02X (SPI state=%d)\n\r", strobe_command, (int)HAL_SPI_GetState(&CC1201_SPI_HANDLE));
    status = HAL_SPI_TransmitReceive(&CC1201_SPI_HANDLE, &strobe_command, &rx_data, 1, 1000);
    printf("[DEBUG] CC1201_SendStrobe 0x%02X -> HAL=%d, statusByte=0x%02X\n\r", strobe_command, status, rx_data);
    
    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_SET); // Pull CS high
    
    if (status == HAL_OK) {
        if (status_byte != NULL) {
            *status_byte = rx_data;
        }
    }
    return status;
}

/**
 * @brief Writes configuration settings to CC1201 registers.
 *
 * @param settings Array of register settings to write.
 * @param num_settings Number of settings in the array.
 * @return HAL_StatusTypeDef Status of the configuration (HAL_OK on success).
 */
HAL_StatusTypeDef CC1201_WriteRegisterConfig(const registerSetting_t *settings, uint16_t num_settings)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    for (uint16_t i = 0; i < num_settings; i++) {
        printf("[DEBUG] CFG[%u]: addr=0x%04X data=0x%02X\n\r", i, settings[i].addr, settings[i].data);
        status = CC1201_WriteRegister(settings[i].addr, settings[i].data);
        if (status != HAL_OK) {
            printf("[DEBUG] CFG write failed at idx=%u HAL=%d\n\r", i, status);
            return status;
        }
        // brief inter-write delay
        for (volatile uint32_t d = 0; d < 500; ++d) { __NOP(); }
    }
    
    return status;
}

/**
 * @brief Gets the number of preferred settings.
 *
 * @return uint16_t Number of settings in the preferred settings array.
 */
uint16_t CC1201_GetNumPreferredSettings(void)
{
    return sizeof(preferredSettings) / sizeof(registerSetting_t);
}

/**
 * @brief Gets a pointer to the preferred settings array.
 *
 * @return const registerSetting_t* Pointer to the preferred settings array.
 */
const registerSetting_t* CC1201_GetPreferredSettings(void)
{
    return preferredSettings;
}