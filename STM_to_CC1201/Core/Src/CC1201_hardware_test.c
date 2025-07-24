/* CC1201 Hardware Diagnostic Test */

#include "main.h"
#include "CC1201_commands.h"
#include "CC1201_simple_link_reg_config.h"
#include "CC1201_reg.h"
#include <stdio.h>

/**
 * @brief Comprehensive hardware diagnostic test for CC1201
 */
void CC1201_HardwareDiagnostic(void) {
    printf("\n=== CC1201 HARDWARE DIAGNOSTIC TEST ===\n\r");
    
    // 1. Test SPI with different patterns
    printf("1. SPI COMMUNICATION PATTERN TEST\n\r");
    
    uint8_t test_commands[] = {0x3D, 0x30, 0x34, 0x36}; // NOP, RESET, IDLE, CALIBRATE
    const char* cmd_names[] = {"NOP", "RESET", "IDLE", "CALIBRATE"};
    
    for (int i = 0; i < 4; i++) {
        uint8_t tx_data = test_commands[i];
        uint8_t rx_data = 0xFF; // Initialize with known pattern
        
        // Manual SPI transaction to see exact response
        HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_StatusTypeDef spi_status = HAL_SPI_TransmitReceive(&CC1201_SPI_HANDLE, &tx_data, &rx_data, 1, 1000);
        HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_SET);
        
        printf("  %s (0x%02X): TX=0x%02X -> RX=0x%02X, HAL=%d\n\r", 
               cmd_names[i], tx_data, tx_data, rx_data, spi_status);
        
        if (rx_data == 0xFF) {
            printf("    ⚠ MISO may be disconnected (received same as initialized)\n\r");
        } else if (rx_data == 0x00) {
            printf("    ⚠ Consistent 0x00 - CC1201 may not be responding\n\r");
        } else {
            printf("    ✓ Got response - CC1201 may be communicating\n\r");
        }
        HAL_Delay(10);
    }
    
    // 2. Test with different SPI settings
    printf("\n2. SPI CONFIGURATION VERIFICATION\n\r");
    printf("  Current SPI2 Settings:\n\r");
    printf("    Mode: 0x%08lX (0x00400000 = Master)\n\r", (unsigned long)CC1201_SPI_HANDLE.Init.Mode);
    printf("    DataSize: 0x%08lX (0x00000007 = 8-bit)\n\r", (unsigned long)CC1201_SPI_HANDLE.Init.DataSize);
    printf("    CPOL: %lu (0 = Low)\n\r", (unsigned long)CC1201_SPI_HANDLE.Init.CLKPolarity);
    printf("    CPHA: %lu (0 = 1st Edge)\n\r", (unsigned long)CC1201_SPI_HANDLE.Init.CLKPhase);
    printf("    BaudRate: 0x%08lX\n\r", (unsigned long)CC1201_SPI_HANDLE.Init.BaudRatePrescaler);
    
    // 3. GPIO Pin State Check
    printf("\n3. GPIO PIN STATE CHECK\n\r");
    GPIO_PinState cs_state = HAL_GPIO_ReadPin(CC1201_CS_PORT, CC1201_CS_PIN);
    printf("  CS Pin (PE4) current state: %s\n\r", cs_state == GPIO_PIN_SET ? "HIGH" : "LOW");
    
    // Toggle CS pin to verify control
    printf("  Testing CS pin control:\n\r");
    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    cs_state = HAL_GPIO_ReadPin(CC1201_CS_PORT, CC1201_CS_PIN);
    printf("    After setting LOW: %s\n\r", cs_state == GPIO_PIN_SET ? "HIGH" : "LOW");
    
    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    cs_state = HAL_GPIO_ReadPin(CC1201_CS_PORT, CC1201_CS_PIN);
    printf("    After setting HIGH: %s\n\r", cs_state == GPIO_PIN_SET ? "HIGH" : "LOW");
    
    // 4. Loopback test (if possible)
    printf("\n4. SPI LOOPBACK TEST (Connect MOSI to MISO temporarily)\n\r");
    uint8_t loopback_patterns[] = {0xAA, 0x55, 0xFF, 0x00, 0xA5};
    
    for (int i = 0; i < 5; i++) {
        uint8_t tx = loopback_patterns[i];
        uint8_t rx = 0x00;
        
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&CC1201_SPI_HANDLE, &tx, &rx, 1, 1000);
        printf("  Pattern 0x%02X: TX=0x%02X -> RX=0x%02X, HAL=%d", tx, tx, rx, status);
        
        if (rx == tx) {
            printf(" ✓ LOOPBACK OK\n\r");
        } else {
            printf(" ✗ No loopback\n\r");
        }
    }
    
    // 5. Diagnostic Summary
    printf("\n5. DIAGNOSTIC SUMMARY\n\r");
    printf("  If all RX data is 0x00:\n\r");
    printf("    - Check CC1201 power supply (3.3V)\n\r");
    printf("    - Check MISO connection (CC1201 SO pin to STM32 PA6)\n\r");
    printf("    - Check if CC1201 is in reset or sleep state\n\r");
    printf("    - Verify CC1201 crystal oscillator is running\n\r");
    printf("\n  If all RX data is 0xFF:\n\r");
    printf("    - MISO line may be floating (not connected)\n\r");
    printf("    - Check SPI MISO pin configuration\n\r");
    printf("\n  If RX data varies:\n\r");
    printf("    - CC1201 is responding - check command format\n\r");
    printf("    - May need proper initialization sequence\n\r");
    
    printf("=== HARDWARE DIAGNOSTIC COMPLETE ===\n\r");
}
