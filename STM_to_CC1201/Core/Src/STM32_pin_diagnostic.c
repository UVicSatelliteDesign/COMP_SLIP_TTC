/* STM32 Pin Configuration Diagnostic Tool */

#include "main.h"
#include <stdio.h>

/**
 * @brief Comprehensive STM32 pin configuration and state diagnostic
 */
void STM32_PinConfigDiagnostic(void) {
    printf("\n=== STM32 PIN CONFIGURATION DIAGNOSTIC ===\n\r");
    
    // 1. SPI2 Pin Configuration Check
    printf("1. SPI2 PIN CONFIGURATION ANALYSIS\n\r");
    
    // Check SPI2 pins (assuming standard STM32H7 Nucleo mapping)
    // SPI2_SCK  = PB10 or PB13 or PA9
    // SPI2_MISO = PB14 or PC2  or PA6  
    // SPI2_MOSI = PB15 or PC3  or PA10
    
    printf("  Checking potential SPI2 pins:\n\r");
    
    // Check PA6 (likely MISO)
    GPIO_TypeDef* porta = GPIOA;
    uint32_t moder_a6 = (porta->MODER >> (6 * 2)) & 0x3;
    uint32_t afr_a6 = (porta->AFR[0] >> (6 * 4)) & 0xF;
    GPIO_PinState state_a6 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
    
    printf("    PA6 (MISO): MODE=%lu (2=AF), AF=%lu (5=SPI2), STATE=%s\n\r", 
           moder_a6, afr_a6, state_a6 == GPIO_PIN_SET ? "HIGH" : "LOW");
    
    // Check PA10 (likely MOSI)  
    uint32_t moder_a10 = (porta->MODER >> (10 * 2)) & 0x3;
    uint32_t afr_a10 = (porta->AFR[1] >> ((10-8) * 4)) & 0xF;
    GPIO_PinState state_a10 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
    
    printf("    PA10 (MOSI): MODE=%lu (2=AF), AF=%lu (5=SPI2), STATE=%s\n\r", 
           moder_a10, afr_a10, state_a10 == GPIO_PIN_SET ? "HIGH" : "LOW");
    
    // Check PB10 (likely SCK)
    GPIO_TypeDef* portb = GPIOB;
    uint32_t moder_b10 = (portb->MODER >> (10 * 2)) & 0x3;
    uint32_t afr_b10 = (portb->AFR[1] >> ((10-8) * 4)) & 0xF;
    GPIO_PinState state_b10 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
    
    printf("    PB10 (SCK): MODE=%lu (2=AF), AF=%lu (5=SPI2), STATE=%s\n\r", 
           moder_b10, afr_b10, state_b10 == GPIO_PIN_SET ? "HIGH" : "LOW");
    
    // 2. CS Pin (PE4) Detailed Analysis
    printf("\n2. CS PIN (PE4) DETAILED ANALYSIS\n\r");
    GPIO_TypeDef* porte = GPIOE;
    uint32_t moder_e4 = (porte->MODER >> (4 * 2)) & 0x3;
    uint32_t otyper_e4 = (porte->OTYPER >> 4) & 0x1;
    uint32_t ospeedr_e4 = (porte->OSPEEDR >> (4 * 2)) & 0x3;
    uint32_t pupdr_e4 = (porte->PUPDR >> (4 * 2)) & 0x3;
    GPIO_PinState state_e4 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
    
    printf("    PE4 Configuration:\n\r");
    printf("      MODE: %lu (0=Input, 1=Output, 2=AF, 3=Analog)\n\r", moder_e4);
    printf("      OTYPE: %lu (0=Push-pull, 1=Open-drain)\n\r", otyper_e4);
    printf("      SPEED: %lu (0=Low, 1=Medium, 2=High, 3=VeryHigh)\n\r", ospeedr_e4);
    printf("      PUPD: %lu (0=None, 1=Pull-up, 2=Pull-down)\n\r", pupdr_e4);
    printf("      STATE: %s\n\r", state_e4 == GPIO_PIN_SET ? "HIGH" : "LOW");
    
    // 3. INT Pin (PD4) Analysis
    printf("\n3. INT PIN (PD4) ANALYSIS\n\r");
    GPIO_TypeDef* portd = GPIOD;
    uint32_t moder_d4 = (portd->MODER >> (4 * 2)) & 0x3;
    uint32_t pupdr_d4 = (portd->PUPDR >> (4 * 2)) & 0x3;
    GPIO_PinState state_d4 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);
    
    printf("    PD4 Configuration:\n\r");
    printf("      MODE: %lu (0=Input, 1=Output, 2=AF, 3=Analog)\n\r", moder_d4);
    printf("      PUPD: %lu (0=None, 1=Pull-up, 2=Pull-down)\n\r", pupdr_d4);
    printf("      STATE: %s\n\r", state_d4 == GPIO_PIN_SET ? "HIGH" : "LOW");
    
    // 4. Alternative Pin Scanning
    printf("\n4. SCANNING FOR POTENTIAL SPI PINS\n\r");
    printf("  Checking all GPIO ports for AF5 (SPI2) configuration:\n\r");
    
    // Port A scan
    printf("    Port A:\n\r");
    for (int pin = 0; pin < 16; pin++) {
        uint32_t mode = (porta->MODER >> (pin * 2)) & 0x3;
        if (mode == 2) { // Alternate function mode
            uint32_t af = (pin < 8) ? 
                         ((porta->AFR[0] >> (pin * 4)) & 0xF) : 
                         ((porta->AFR[1] >> ((pin-8) * 4)) & 0xF);
            if (af == 5) { // AF5 = SPI2
                GPIO_PinState state = HAL_GPIO_ReadPin(GPIOA, 1 << pin);
                printf("      PA%d: AF5 (SPI2), STATE=%s\n\r", pin, 
                       state == GPIO_PIN_SET ? "HIGH" : "LOW");
            }
        }
    }
    
    // Port B scan
    printf("    Port B:\n\r");
    for (int pin = 0; pin < 16; pin++) {
        uint32_t mode = (portb->MODER >> (pin * 2)) & 0x3;
        if (mode == 2) { // Alternate function mode
            uint32_t af = (pin < 8) ? 
                         ((portb->AFR[0] >> (pin * 4)) & 0xF) : 
                         ((portb->AFR[1] >> ((pin-8) * 4)) & 0xF);
            if (af == 5) { // AF5 = SPI2
                GPIO_PinState state = HAL_GPIO_ReadPin(GPIOB, 1 << pin);
                printf("      PB%d: AF5 (SPI2), STATE=%s\n\r", pin, 
                       state == GPIO_PIN_SET ? "HIGH" : "LOW");
            }
        }
    }
    
    // Port C scan
    GPIO_TypeDef* portc = GPIOC;
    printf("    Port C:\n\r");
    for (int pin = 0; pin < 16; pin++) {
        uint32_t mode = (portc->MODER >> (pin * 2)) & 0x3;
        if (mode == 2) { // Alternate function mode
            uint32_t af = (pin < 8) ? 
                         ((portc->AFR[0] >> (pin * 4)) & 0xF) : 
                         ((portc->AFR[1] >> ((pin-8) * 4)) & 0xF);
            if (af == 5) { // AF5 = SPI2
                GPIO_PinState state = HAL_GPIO_ReadPin(GPIOC, 1 << pin);
                printf("      PC%d: AF5 (SPI2), STATE=%s\n\r", pin, 
                       state == GPIO_PIN_SET ? "HIGH" : "LOW");
            }
        }
    }
    
    // 5. SPI2 Peripheral Register Analysis
    printf("\n5. SPI2 PERIPHERAL REGISTER ANALYSIS\n\r");
    printf("    SPI2->CR1: 0x%08lX\n\r", (unsigned long)SPI2->CR1);
    printf("    SPI2->CR2: 0x%08lX\n\r", (unsigned long)SPI2->CR2);
    printf("    SPI2->SR:  0x%08lX\n\r", (unsigned long)SPI2->SR);
    printf("    SPI2->CFG1: 0x%08lX\n\r", (unsigned long)SPI2->CFG1);
    printf("    SPI2->CFG2: 0x%08lX\n\r", (unsigned long)SPI2->CFG2);
    
    // Decode important bits
    uint32_t cr1 = SPI2->CR1;
    printf("    SPI2 Decoded:\n\r");
    printf("      SPE (Enable): %s\n\r", (cr1 & SPI_CR1_SPE) ? "ENABLED" : "DISABLED");
    printf("      MSTR (Master): %s\n\r", (cr1 & SPI_CR1_MSTR) ? "MASTER" : "SLAVE");
    printf("      SSI (Internal SS): %s\n\r", (cr1 & SPI_CR1_SSI) ? "HIGH" : "LOW");
    printf("      SSM (Software SS): %s\n\r", (cr1 & SPI_CR1_SSM) ? "ENABLED" : "DISABLED");
    
    // 6. Clock Configuration Check
    printf("\n6. CLOCK CONFIGURATION CHECK\n\r");
    uint32_t rcc_apb1enr = RCC->APB1LENR;
    printf("    SPI2 Clock: %s\n\r", (rcc_apb1enr & RCC_APB1LENR_SPI2EN) ? "ENABLED" : "DISABLED");
    
    uint32_t rcc_ahb4enr = RCC->AHB4ENR;
    printf("    GPIOA Clock: %s\n\r", (rcc_ahb4enr & RCC_AHB4ENR_GPIOAEN) ? "ENABLED" : "DISABLED");
    printf("    GPIOB Clock: %s\n\r", (rcc_ahb4enr & RCC_AHB4ENR_GPIOBEN) ? "ENABLED" : "DISABLED");
    printf("    GPIOC Clock: %s\n\r", (rcc_ahb4enr & RCC_AHB4ENR_GPIOCEN) ? "ENABLED" : "DISABLED");
    printf("    GPIOD Clock: %s\n\r", (rcc_ahb4enr & RCC_AHB4ENR_GPIODEN) ? "ENABLED" : "DISABLED");
    printf("    GPIOE Clock: %s\n\r", (rcc_ahb4enr & RCC_AHB4ENR_GPIOEEN) ? "ENABLED" : "DISABLED");
    
    printf("=== PIN CONFIGURATION DIAGNOSTIC COMPLETE ===\n\r");
}
