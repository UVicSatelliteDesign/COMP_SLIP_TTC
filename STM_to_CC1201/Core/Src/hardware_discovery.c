/* Hardware Discovery Tool - Find actual CC1201 connections */

#include "main.h"
#include <stdio.h>
#include <stdbool.h>

/**
 * @brief Scan all GPIO pins to find which ones are actually connected
 * This will help us identify the real CC1201 pin connections
 */
void Hardware_Discovery_Scan(void) {
    printf("\n🔍 HARDWARE DISCOVERY - FINDING ACTUAL CC1201 CONNECTIONS 🔍\n\r");
    printf("=================================================================\n\r");
    
    // Enable all GPIO clocks first
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    
    printf("\n1. SCANNING ALL GPIO PINS FOR ACTIVITY...\n\r");
    printf("   (Looking for pins that respond differently when toggled)\n\r");
    
    // Arrays to store pin information
    GPIO_TypeDef* ports[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH};
    const char* port_names[] = {"PA", "PB", "PC", "PD", "PE", "PF", "PG", "PH"};
    uint8_t active_count = 0;
    
    for (int port = 0; port < 8; port++) {
        for (int pin = 0; pin < 16; pin++) {
            uint16_t gpio_pin = (1 << pin);
            
            // Configure pin as output first
            GPIO_InitTypeDef GPIO_InitStruct = {0};
            GPIO_InitStruct.Pin = gpio_pin;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            HAL_GPIO_Init(ports[port], &GPIO_InitStruct);
            
            // Test if pin can be controlled
            HAL_GPIO_WritePin(ports[port], gpio_pin, GPIO_PIN_SET);
            HAL_Delay(1);
            GPIO_PinState state_high = HAL_GPIO_ReadPin(ports[port], gpio_pin);
            
            HAL_GPIO_WritePin(ports[port], gpio_pin, GPIO_PIN_RESET);
            HAL_Delay(1);
            GPIO_PinState state_low = HAL_GPIO_ReadPin(ports[port], gpio_pin);
            
            // If pin responds to control, it might be connected
            if (state_high == GPIO_PIN_SET && state_low == GPIO_PIN_RESET) {
                printf("   %s%d: Controllable ✓\n\r", port_names[port], pin);
                active_count++;
            }
            
            // Return pin to analog mode to avoid interference
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            HAL_GPIO_Init(ports[port], &GPIO_InitStruct);
        }
    }
    
    printf("   Found %d controllable pins\n\r", active_count);
    
    printf("\n2. TESTING SPI COMMUNICATION ON DIFFERENT PIN COMBINATIONS...\n\r");
    
    // Test different potential SPI pin combinations
    // Based on common STM32H7 SPI2 alternate pin mappings
    
    typedef struct {
        GPIO_TypeDef* miso_port;
        uint16_t miso_pin;
        GPIO_TypeDef* mosi_port;
        uint16_t mosi_pin;
        GPIO_TypeDef* sck_port;
        uint16_t sck_pin;
        const char* description;
    } spi_pin_combo_t;
    
    spi_pin_combo_t combos[] = {
        {GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_10, GPIOB, GPIO_PIN_10, "Current: PA6/PA10/PB10"},
        {GPIOC, GPIO_PIN_2, GPIOA, GPIO_PIN_10, GPIOB, GPIO_PIN_10, "Alt1: PC2/PA10/PB10"},
        {GPIOB, GPIO_PIN_14, GPIOA, GPIO_PIN_10, GPIOB, GPIO_PIN_10, "Alt2: PB14/PA10/PB10"},
        {GPIOA, GPIO_PIN_6, GPIOC, GPIO_PIN_1, GPIOB, GPIO_PIN_10, "Alt3: PA6/PC1/PB10"},
        {GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_10, GPIOB, GPIO_PIN_13, "Alt4: PA6/PA10/PB13"},
        {GPIOC, GPIO_PIN_2, GPIOC, GPIO_PIN_1, GPIOB, GPIO_PIN_10, "Alt5: PC2/PC1/PB10"},
        {GPIOB, GPIO_PIN_14, GPIOC, GPIO_PIN_3, GPIOB, GPIO_PIN_10, "Alt6: PB14/PC3/PB10"},
    };
    
    for (int combo = 0; combo < 7; combo++) {
        printf("   Testing %s...\n\r", combos[combo].description);
        
        // Disable SPI2 first
        __HAL_SPI_DISABLE(&hspi2);
        
        // Configure pins for this combination
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        
        // MISO
        GPIO_InitStruct.Pin = combos[combo].miso_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(combos[combo].miso_port, &GPIO_InitStruct);
        
        // MOSI
        GPIO_InitStruct.Pin = combos[combo].mosi_pin;
        HAL_GPIO_Init(combos[combo].mosi_port, &GPIO_InitStruct);
        
        // SCK
        GPIO_InitStruct.Pin = combos[combo].sck_pin;
        HAL_GPIO_Init(combos[combo].sck_port, &GPIO_InitStruct);
        
        // Re-enable SPI2
        __HAL_SPI_ENABLE(&hspi2);
        
        // Test this configuration
        uint8_t test_patterns[] = {0xAA, 0x55, 0xFF, 0x30}; // Include CC1201 RESET command
        uint8_t combo_working = 0; // Use 0/1 instead of bool
        
        for (int p = 0; p < 4; p++) {
            uint8_t tx_data = test_patterns[p];
            uint8_t rx_data = 0x00;
            
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); // CS LOW
            HAL_SPI_TransmitReceive(&hspi2, &tx_data, &rx_data, 1, 1000);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);   // CS HIGH
            
            printf("     Pattern 0x%02X: TX=0x%02X -> RX=0x%02X", test_patterns[p], tx_data, rx_data);
            
            if (rx_data != 0x00 && rx_data != 0xFF) {
                printf(" ✅ RESPONSE!\n\r");
                combo_working = 1;
            } else {
                printf("\n\r");
            }
        }
        
        if (combo_working) {
            printf("   🎯 FOUND WORKING COMBINATION: %s\n\r", combos[combo].description);
            printf("   This appears to be the correct SPI pin mapping!\n\r");
            return; // Keep this configuration
        }
        
        // Reset pins to analog if this combo didn't work
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pin = combos[combo].miso_pin;
        HAL_GPIO_Init(combos[combo].miso_port, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = combos[combo].mosi_pin;
        HAL_GPIO_Init(combos[combo].mosi_port, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = combos[combo].sck_pin;
        HAL_GPIO_Init(combos[combo].sck_port, &GPIO_InitStruct);
    }
    
    printf("\n3. DETAILED PIN ANALYSIS FOR POTENTIAL CC1201 CONNECTIONS...\n\r");
    
    // Test common pins that might be connected to CC1201
    uint16_t test_pins[] = {
        // Port A
        (0 << 8) | 0, (0 << 8) | 1, (0 << 8) | 2, (0 << 8) | 3, (0 << 8) | 4, (0 << 8) | 5,
        (0 << 8) | 6, (0 << 8) | 7, (0 << 8) | 8, (0 << 8) | 9, (0 << 8) | 10, (0 << 8) | 11,
        // Port B
        (1 << 8) | 0, (1 << 8) | 1, (1 << 8) | 2, (1 << 8) | 3, (1 << 8) | 4, (1 << 8) | 5,
        (1 << 8) | 10, (1 << 8) | 12, (1 << 8) | 13, (1 << 8) | 14, (1 << 8) | 15,
        // Port C
        (2 << 8) | 0, (2 << 8) | 1, (2 << 8) | 2, (2 << 8) | 3, (2 << 8) | 4, (2 << 8) | 5,
        // Port D
        (3 << 8) | 0, (3 << 8) | 1, (3 << 8) | 2, (3 << 8) | 3,
        // Port E
        (4 << 8) | 0, (4 << 8) | 1, (4 << 8) | 2, (4 << 8) | 3, (4 << 8) | 4, (4 << 8) | 5
    };
    
    for (int i = 0; i < sizeof(test_pins)/sizeof(test_pins[0]); i++) {
        uint8_t port_idx = test_pins[i] >> 8;
        uint8_t pin_idx = test_pins[i] & 0xFF;
        uint16_t gpio_pin = (1 << pin_idx);
        
        // Test as potential MISO (input from CC1201)
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = gpio_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(ports[port_idx], &GPIO_InitStruct);
        
        GPIO_PinState state_pullup = HAL_GPIO_ReadPin(ports[port_idx], gpio_pin);
        
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(ports[port_idx], &GPIO_InitStruct);
        
        GPIO_PinState state_pulldown = HAL_GPIO_ReadPin(ports[port_idx], gpio_pin);
        
        if (state_pullup != state_pulldown) {
            printf("   %s%d: Pull-up=%d, Pull-down=%d - Potentially connected\n\r", 
                   port_names[port_idx], pin_idx, state_pullup, state_pulldown);
        }
    }
    
    printf("\n4. FINAL RECOMMENDATION...\n\r");
    printf("   Based on the scan results above:\n\r");
    printf("   - Look for pins that showed 'RESPONSE!' in SPI tests\n\r");
    printf("   - Check pins that behave differently with pull-up/pull-down\n\r");
    printf("   - These are likely your actual CC1201 connections\n\r");
    
    // Restore original SPI configuration
    printf("\n5. RESTORING ORIGINAL CONFIGURATION...\n\r");
    __HAL_SPI_DISABLE(&hspi2);
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // Restore PA6 (MISO)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Restore PA10 (MOSI)
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Restore PB10 (SCK)
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    __HAL_SPI_ENABLE(&hspi2);
    
    printf("=== HARDWARE DISCOVERY COMPLETE ===\n\r");
}
