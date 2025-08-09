/* Manual SPI2 Pin Configuration Fix */

#include "main.h"
#include <stdio.h>

/**
 * @brief Manually configure SPI2 pins with correct alternate function
 * This fixes the issue where pins are in ANALOG mode instead of AF mode
 */
void Manual_SPI2_Pin_Config(void) {
    printf("\n=== MANUAL SPI2 PIN CONFIGURATION ===\n\r");
    
    // Enable GPIO clocks (should already be enabled, but make sure)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 1. Configure PA6 as SPI2_MISO
    printf("1. Configuring PA6 (MISO) for SPI2_AF5...\n\r");
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;          // Alternate Function Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;              // No pull-up/down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;    // High speed
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;       // AF5 = SPI2
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Verify PA6 configuration
    uint32_t pa6_mode = (GPIOA->MODER >> 12) & 0x3;
    uint32_t pa6_af = (GPIOA->AFR[0] >> 24) & 0xF;
    printf("    PA6 after config: MODE=%lu (should be 2), AF=%lu (should be 5)\n\r", pa6_mode, pa6_af);
    
    // 2. Configure PA10 as SPI2_MOSI  
    printf("2. Configuring PA10 (MOSI) for SPI2_AF5...\n\r");
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;          // Alternate Function Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;              // No pull-up/down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;    // High speed
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;       // AF5 = SPI2
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Verify PA10 configuration
    uint32_t pa10_mode = (GPIOA->MODER >> 20) & 0x3;
    uint32_t pa10_af = (GPIOA->AFR[1] >> 8) & 0xF;
    printf("    PA10 after config: MODE=%lu (should be 2), AF=%lu (should be 5)\n\r", pa10_mode, pa10_af);
    
    // 3. Configure PB10 as SPI2_SCK
    printf("3. Configuring PB10 (SCK) for SPI2_AF5...\n\r");
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;          // Alternate Function Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;              // No pull-up/down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;    // High speed
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;       // AF5 = SPI2
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // Verify PB10 configuration
    uint32_t pb10_mode = (GPIOB->MODER >> 20) & 0x3;
    uint32_t pb10_af = (GPIOB->AFR[1] >> 8) & 0xF;
    printf("    PB10 after config: MODE=%lu (should be 2), AF=%lu (should be 5)\n\r", pb10_mode, pb10_af);
    
    // 4. Verify PE4 (CS) is still correctly configured as GPIO output
    printf("4. Verifying PE4 (CS) configuration...\n\r");
    uint32_t pe4_mode = (GPIOE->MODER >> 8) & 0x3;
    printf("    PE4 mode: %lu (should be 1 for GPIO output)\n\r", pe4_mode);
    
    // Make sure CS starts HIGH (inactive)
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
    printf("    PE4 set to HIGH (CS inactive)\n\r");
    
    // 5. Final verification
    printf("\n5. FINAL PIN STATE VERIFICATION:\n\r");
    printf("    PA6 (MISO): MODE=%lu, STATE=%s\n\r", 
           (GPIOA->MODER >> 12) & 0x3,
           HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) ? "HIGH" : "LOW");
    printf("    PA10 (MOSI): MODE=%lu, STATE=%s\n\r", 
           (GPIOA->MODER >> 20) & 0x3,
           HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) ? "HIGH" : "LOW");
    printf("    PB10 (SCK): MODE=%lu, STATE=%s\n\r", 
           (GPIOB->MODER >> 20) & 0x3,
           HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) ? "HIGH" : "LOW");
    printf("    PE4 (CS): MODE=%lu, STATE=%s\n\r", 
           (GPIOE->MODER >> 8) & 0x3,
           HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) ? "HIGH" : "LOW");
    
    printf("=== MANUAL PIN CONFIGURATION COMPLETE ===\n\r");
    
    // Small delay to let pins settle
    HAL_Delay(10);
}
