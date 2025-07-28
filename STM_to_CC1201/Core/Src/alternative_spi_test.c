/* Alternative SPI2 Pin Mapping Test */

#include "main.h"
#include <stdio.h>

/**
 * @brief Test different possible SPI2 MISO pin configurations
 * STM32H7 supports multiple pin options for SPI2
 */
void Test_Alternative_SPI2_Pins(void) {
    printf("\n=== TESTING ALTERNATIVE SPI2 PIN MAPPINGS ===\n\r");
    
    // Store original SPI configuration
    printf("1. Testing current configuration (PA6 MISO)...\n\r");
    uint8_t tx_data = 0xAA;
    uint8_t rx_data = 0x00;
    
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); // CS LOW
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi4, &tx_data, &rx_data, 1, 1000);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);   // CS HIGH
    
    printf("    Current PA6: TX=0x%02X -> RX=0x%02X, HAL=%d\n\r", tx_data, rx_data, status);
    
    // Test 2: Try PC2 as MISO (alternative SPI2 mapping)
    printf("\n2. Testing PC2 as SPI2_MISO...\n\r");
    
    // Disable SPI2 first
    __HAL_SPI_DISABLE(&hspi4);
    
    // Reconfigure PA6 back to analog to disconnect it
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Enable GPIOC clock
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // Configure PC2 as SPI2_MISO
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // Re-enable SPI2
    __HAL_SPI_ENABLE(&hspi4);
    
    // Test with PC2
    rx_data = 0x00;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); // CS LOW
    status = HAL_SPI_TransmitReceive(&hspi4, &tx_data, &rx_data, 1, 1000);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);   // CS HIGH
    
    printf("    PC2 test: TX=0x%02X -> RX=0x%02X, HAL=%d\n\r", tx_data, rx_data, status);
    
    if (rx_data != 0x00) {
        printf("    ✅ PC2 is working as MISO!\n\r");
        return; // Keep PC2 configuration
    }
    
    // Test 3: Try PB14 as MISO (another alternative)
    printf("\n3. Testing PB14 as SPI2_MISO...\n\r");
    
    // Disable SPI2
    __HAL_SPI_DISABLE(&hspi4);
    
    // Disconnect PC2
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // Configure PB14 as SPI2_MISO
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // Re-enable SPI2
    __HAL_SPI_ENABLE(&hspi4);
    
    // Test with PB14
    rx_data = 0x00;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); // CS LOW
    status = HAL_SPI_TransmitReceive(&hspi4, &tx_data, &rx_data, 1, 1000);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);   // CS HIGH
    
    printf("    PB14 test: TX=0x%02X -> RX=0x%02X, HAL=%d\n\r", tx_data, rx_data, status);
    
    if (rx_data != 0x00) {
        printf("    ✅ PB14 is working as MISO!\n\r");
        return; // Keep PB14 configuration
    }
    
    // Test 4: Check if any pin can be used as MISO via scanning
    printf("\n4. SCANNING POTENTIAL MISO PINS...\n\r");
    printf("   Testing pins that could potentially be connected to CC1201 MISO:\n\r");
    
    // Test some common pins that might be wired to CC1201
    uint16_t test_pins[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9};
    GPIO_TypeDef* test_ports[] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA};
    const char* pin_names[] = {"PA0", "PA1", "PA4", "PA5", "PA7", "PA8", "PA9"};
    
    for (int i = 0; i < 7; i++) {
        // Disable SPI2
        __HAL_SPI_DISABLE(&hspi4);
        
        // Disconnect previous pin
        GPIO_InitStruct.Pin = GPIO_PIN_14;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        
        // Try current pin as MISO
        GPIO_InitStruct.Pin = test_pins[i];
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(test_ports[i], &GPIO_InitStruct);
        
        // Re-enable SPI2
        __HAL_SPI_ENABLE(&hspi4);
        
        // Test
        rx_data = 0x00;
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
        status = HAL_SPI_TransmitReceive(&hspi4, &tx_data, &rx_data, 1, 1000);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
        
        printf("   %s: TX=0x%02X -> RX=0x%02X", pin_names[i], tx_data, rx_data);
        if (rx_data != 0x00) {
            printf(" ✅ WORKING!\n\r");
            printf("   🎯 FOUND: %s can receive SPI data!\n\r", pin_names[i]);
            return;
        } else {
            printf("\n\r");
        }
    }
    
    // If we get here, restore original PA6 configuration
    printf("\n5. RESTORING ORIGINAL PA6 CONFIGURATION...\n\r");
    __HAL_SPI_DISABLE(&hspi4);
    
    // Restore PA6 as MISO
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    __HAL_SPI_ENABLE(&hspi4);
    
    printf("=== ALTERNATIVE PIN TEST COMPLETE ===\n\r");
    printf("CONCLUSION: No working MISO pin found. Possible issues:\n\r");
    printf("  • CC1201 not physically connected to any STM32 pin\n\r");
    printf("  • CC1201 MISO/SO pin not connected\n\r");
    printf("  • CC1201 power supply issue\n\r");
    printf("  • Hardware fault in CC1201 or STM32 board\n\r");
}
