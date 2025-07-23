#include "main.h"
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

    printf("    [DEBUG] SendStrobe called with command 0x%02X\n\r", strobe_command);

    // Debug: Check CS pin before transaction
    printf("  Strobe 0x%02X: CS before=", strobe_command);
    printf("%s", HAL_GPIO_ReadPin(CC1201_CS_PORT, CC1201_CS_PIN) == GPIO_PIN_SET ? "HIGH" : "LOW");

    printf("    [DEBUG] About to pull CS low...\n\r");
    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_RESET); // Pull CS low
    
    // Debug: Check CS pin during transaction
    printf(", CS during=");
    printf("%s", HAL_GPIO_ReadPin(CC1201_CS_PORT, CC1201_CS_PIN) == GPIO_PIN_SET ? "HIGH" : "LOW");
    
    printf("    [DEBUG] About to start SPI transaction...\n\r");
    
    // Check SPI configuration before transaction
    printf("    [SPI DEBUG] SPI2 Config: State=%d, Mode=%lu, DataSize=%lu\n\r", 
           CC1201_SPI_HANDLE.State, 
           CC1201_SPI_HANDLE.Init.Mode,
           CC1201_SPI_HANDLE.Init.DataSize);
    printf("    [SPI DEBUG] BaudRate=%lu, CPOL=%lu, CPHA=%lu\n\r",
           CC1201_SPI_HANDLE.Init.BaudRatePrescaler,
           CC1201_SPI_HANDLE.Init.CLKPolarity,
           CC1201_SPI_HANDLE.Init.CLKPhase);
    
    // Small delay to ensure CS is stable
    HAL_Delay(1);

    // Use even shorter timeout to avoid long waits
    status = HAL_SPI_TransmitReceive(&CC1201_SPI_HANDLE, &strobe_command, &rx_data, 1, 100); // 100ms timeout
    
    printf("    [DEBUG] SPI transaction returned with status: %d\n\r", status);
    
    // If TransmitReceive failed, try separate transmit and receive
    if (status != HAL_OK) {
        printf("    [DEBUG] Trying separate TX/RX...\n\r");
        rx_data = 0x00; // Reset rx_data
        
        status = HAL_SPI_Transmit(&CC1201_SPI_HANDLE, &strobe_command, 1, 100);
        printf("    [DEBUG] SPI Transmit status: %d\n\r", status);
        
        if (status == HAL_OK) {
            status = HAL_SPI_Receive(&CC1201_SPI_HANDLE, &rx_data, 1, 100);
            printf("    [DEBUG] SPI Receive status: %d, RX: 0x%02X\n\r", status, rx_data);
        }
    }
    
    if (status != HAL_OK) {
        printf("    [ERROR] SPI transaction failed! Status: %d\n\r", status);
        if (status == HAL_TIMEOUT) {
            printf("    [ERROR] SPI TIMEOUT occurred!\n\r");
        }
        if (status == HAL_ERROR) {
            printf("    [ERROR] SPI ERROR occurred!\n\r");
        }
        if (status == HAL_BUSY) {
            printf("    [ERROR] SPI BUSY occurred!\n\r");
        }
    }

    printf("    [DEBUG] SPI transaction complete, pulling CS high...\n\r");
    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_SET); // Pull CS high

    // Debug output
    printf(", CS after=HIGH, RX=0x%02X, HAL=%d\n\r", rx_data, status);
    printf("    [DEBUG] SendStrobe complete\n\r");

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
        if (settings[i].addr > 0xFF) {
            // Extended register (requires burst access)
            // For now, skip extended registers in this simple implementation
            continue;
        }
        
        status = CC1201_WriteRegister((uint8_t)settings[i].addr, settings[i].data);
        if (status != HAL_OK) {
            return status;
        }
        
        HAL_Delay(1); // Small delay between register writes
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