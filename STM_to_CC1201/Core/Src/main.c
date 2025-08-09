/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CC1201_commands.h"
#include "CC1201_simple_link_reg_config.h"
#include "CC1201_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;

SPI_HandleTypeDef hspi4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function to decode and print CC1201 status
void print_cc1201_status(uint8_t status_byte, const char* context) {
    uint8_t radio_state = (status_byte >> 4) & 0x0F;
    uint8_t fifo_bytes = status_byte & 0x0F;
    
    printf("  %s: Status=0x%02X, State=0x%X", context, status_byte, radio_state);
    switch(radio_state) {
        case 0x0: printf("(IDLE)"); break;
        case 0x1: printf("(RX)"); break;
        case 0x2: printf("(TX)"); break;
        case 0x3: printf("(FSTXON)"); break;
        case 0x4: printf("(CALIBRATE)"); break;
        case 0x5: printf("(SETTLING)"); break;
        case 0x6: printf("(RX_FIFO_ERR)"); break;
        case 0x7: printf("(TX_FIFO_ERR)"); break;
        default: printf("(UNKNOWN)"); break;
    }
    printf(", FIFO=%d\n\r", fifo_bytes);
}

// Hardware Reset Function
void cc1201_hardware_reset(void) {
    printf("  Performing hardware reset...\n\r");
    
    // Pull reset pin LOW (active reset)
    HAL_GPIO_WritePin(CC1201_RESET_PORT, CC1201_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(10); // Hold reset for 10ms
    
    // Release reset pin HIGH (inactive)
    HAL_GPIO_WritePin(CC1201_RESET_PORT, CC1201_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(50); // Wait for reset to complete
    
    printf("  Hardware reset complete\n\r");
}

// CC1201 Power-up and Reset Sequence  
void cc1201_power_up_sequence(void) {
    printf("\n=== CC1201 POWER-UP SEQUENCE ===\n\r");
    
    // 1. Ensure CS is high (inactive)
    printf("1. Setting CS HIGH (inactive)...\n\r");
    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
    
    // 2. Hardware reset sequence
    printf("2. Hardware reset sequence...\n\r");
    cc1201_hardware_reset();
    
    // 3. Power-up delay (let voltages stabilize)
    printf("3. Power stabilization delay (100ms)...\n\r");
    HAL_Delay(100);
    
    // 4. Crystal oscillator startup time
    printf("4. Crystal oscillator startup delay (10ms)...\n\r");
    HAL_Delay(10);
    
    // 5. Test basic communication after hardware reset
    printf("5. Testing communication after hardware reset...\n\r");
    uint8_t post_hw_reset_status = 0xFF;
    HAL_StatusTypeDef post_hw_reset_result = CC1201_Nop(&post_hw_reset_status);
    printf("  Post-HW-reset NOP: HAL=%d, Status=0x%02X\n\r", post_hw_reset_result, post_hw_reset_status);
    
    // 6. Try software reset sequence (if hardware reset helped)
    if (post_hw_reset_status != 0xFF) {
        printf("6. Attempting software reset...\n\r");
        uint8_t reset_status = 0xFF;
        HAL_StatusTypeDef reset_result = CC1201_SoftReset(&reset_status);
        printf("  Software reset: HAL=%d, Status=0x%02X\n\r", reset_result, reset_status);
        
        // Wait for reset to complete
        HAL_Delay(50);
        
        // Test again after software reset
        uint8_t post_reset_status = 0xFF;
        HAL_StatusTypeDef post_reset_result = CC1201_Nop(&post_reset_status);
        printf("  Post-SW-reset NOP: HAL=%d, Status=0x%02X\n\r", post_reset_result, post_reset_status);
        
        if (post_reset_status != 0xFF && post_reset_status != 0x00) {
            printf("  ✓ CC1201 responding after complete reset sequence!\n\r");
        } else {
            printf("  ✗ CC1201 still not responding after software reset\n\r");
        }
    } else {
        printf("6. Skipping software reset - hardware reset didn't help\n\r");
        printf("  ✗ CC1201 not responding - likely hardware issue\n\r");
    }
    
    printf("=== POWER-UP SEQUENCE COMPLETE ===\n\r");
}

// Comprehensive CC1201 Hardware Diagnostic
void comprehensive_cc1201_diagnostic(void) {
    printf("\n=== COMPREHENSIVE CC1201 HARDWARE DIAGNOSTIC ===\n\r");
    
    // 1. Check pin states BEFORE any SPI communication
    printf("1. INITIAL PIN STATE CHECK:\n\r");
    printf("  CS Pin (PE11): %s\n\r", HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) ? "HIGH" : "LOW");
    printf("  SCK Pin (PE12): %s\n\r", HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12) ? "HIGH" : "LOW");
    printf("  MISO Pin (PE13): %s\n\r", HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) ? "HIGH" : "LOW");
    printf("  MOSI Pin (PE14): %s\n\r", HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) ? "HIGH" : "LOW");
    printf("  RESET Pin (PD4): %s\n\r", HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) ? "HIGH" : "LOW");
    printf("  INT Pin (PD5): %s\n\r", HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) ? "HIGH" : "LOW");
    
    // 2. Manual CS control test
    printf("\n2. MANUAL CS CONTROL TEST:\n\r");
    printf("  Setting CS LOW...\n\r");
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(10);
    printf("  CS Pin: %s\n\r", HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) ? "HIGH" : "LOW");
    printf("  MISO Pin: %s\n\r", HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) ? "HIGH" : "LOW");
    
    printf("  Setting CS HIGH...\n\r");
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(10);
    printf("  CS Pin: %s\n\r", HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) ? "HIGH" : "LOW");
    printf("  MISO Pin: %s\n\r", HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) ? "HIGH" : "LOW");
    
    // 3. Test different SPI byte patterns with CS control
    printf("\n3. RAW SPI BYTE PATTERN TEST:\n\r");
    uint8_t test_patterns[] = {0x00, 0xFF, 0x55, 0xAA, 0x3D}; // Last one is NOP command
    const char* pattern_names[] = {"0x00", "0xFF", "0x55", "0xAA", "NOP(0x3D)"};
    
    for (int i = 0; i < 5; i++) {
        uint8_t tx_data = test_patterns[i];
        uint8_t rx_data = 0x00; // Initialize to 0 this time
        
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); // CS LOW
        HAL_Delay(1);
        HAL_StatusTypeDef spi_status = HAL_SPI_TransmitReceive(&hspi4, &tx_data, &rx_data, 1, 1000);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);   // CS HIGH
        
        printf("  %s: TX=0x%02X -> RX=0x%02X (HAL=%d)\n\r", 
               pattern_names[i], tx_data, rx_data, spi_status);
        
        if (rx_data == tx_data) {
            printf("    ⚠ Echo detected - MISO may be shorted to MOSI\n\r");
        } else if (rx_data == 0xFF) {
            printf("    ⚠ MISO stuck HIGH - check power/connections\n\r");
        } else if (rx_data == 0x00) {
            printf("    ⚠ MISO stuck LOW - check power/connections\n\r");
        } else {
            printf("    ✓ Different response - potential communication\n\r");
        }
        HAL_Delay(10);
    }
    
    // 4. Hardware reset test
    printf("\n4. HARDWARE RESET TEST:\n\r");
    printf("  ✓ Hardware RESET pin configured on PD4\n\r");
    printf("  Testing hardware reset sequence...\n\r");
    
    // Test reset pin control
    printf("  Current RESET pin state: %s\n\r", HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) ? "HIGH" : "LOW");
    printf("  Performing reset pulse...\n\r");
    
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); // Assert reset
    HAL_Delay(10);
    printf("  RESET pin during reset: %s\n\r", HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) ? "HIGH" : "LOW");
    printf("  MISO pin during reset: %s\n\r", HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) ? "HIGH" : "LOW");
    
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);   // Release reset
    HAL_Delay(50);
    printf("  RESET pin after reset: %s\n\r", HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) ? "HIGH" : "LOW");
    printf("  MISO pin after reset: %s\n\r", HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) ? "HIGH" : "LOW");
    
    // Test communication after reset
    uint8_t reset_test_tx = 0x3D, reset_test_rx = 0x00;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_StatusTypeDef reset_comm_status = HAL_SPI_TransmitReceive(&hspi4, &reset_test_tx, &reset_test_rx, 1, 1000);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
    printf("  Post-reset communication: TX=0x%02X -> RX=0x%02X (HAL=%d)\n\r", 
           reset_test_tx, reset_test_rx, reset_comm_status);
    
    printf("\n5. POWER CONSIDERATIONS:\n\r");
    printf("  ⚠ No power enable pin defined in current configuration\n\r");
    printf("  → Check if CC1201 has separate VDD supply\n\r");
    printf("  → Verify CC1201 crystal oscillator is working\n\r");
    
    // 6. Try different SPI clock speeds
    printf("\n6. SPI CLOCK SPEED TEST:\n\r");
    printf("  Current SPI prescaler: %lu\n\r", (unsigned long)hspi4.Init.BaudRatePrescaler);
    
    // Test with slower clock
    printf("  Testing with slower clock (prescaler 32)...\n\r");
    hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    if (HAL_SPI_Init(&hspi4) == HAL_OK) {
        uint8_t tx_slow = 0x3D, rx_slow = 0;
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_StatusTypeDef slow_status = HAL_SPI_TransmitReceive(&hspi4, &tx_slow, &rx_slow, 1, 1000);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
        printf("    Slow NOP: TX=0x%02X -> RX=0x%02X (HAL=%d)\n\r", tx_slow, rx_slow, slow_status);
        
        // Restore original speed
        hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        HAL_SPI_Init(&hspi4);
    }
    
    // 7. Final pin state check
    printf("\n7. FINAL PIN STATE CHECK:\n\r");
    printf("  CS Pin (PE11): %s\n\r", HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) ? "HIGH" : "LOW");
    printf("  MISO Pin (PE13): %s\n\r", HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) ? "HIGH" : "LOW");
    printf("  INT Pin (PD5): %s\n\r", HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) ? "HIGH" : "LOW");
    
    printf("=== HARDWARE DIAGNOSTIC COMPLETE ===\n\r");
    printf("\nRECOMMENDATIONS:\n\r");
    printf("→ If MISO is stuck HIGH: Check CC1201 power supply\n\r");
    printf("→ If MISO is stuck LOW: Check ground connections\n\r");
    printf("→ If echoing MOSI: Check for short circuits\n\r");
    printf("→ Consider adding hardware reset pin control\n\r");
    printf("→ Verify CC1201 crystal oscillator is populated and working\n\r");
    printf("→ Check CC1201 datasheet for proper power-up sequence\n\r");
}

// Test 1: Buffer Read/Write Operations
void test_buffer_operations(void) {
    printf("\n=== TEST 1: BUFFER READ/WRITE OPERATIONS ===\n\r");
    
    HAL_StatusTypeDef status;
    uint8_t status_byte = 0;
    
    // First, ensure we're in IDLE state and flush FIFOs
    printf("1. Preparing for buffer tests...\n\r");
    status = CC1201_EnterIdleMode(&status_byte);
    if (status == HAL_OK) {
        print_cc1201_status(status_byte, "IDLE_MODE");
    }
    
    // Flush both FIFOs
    CC1201_FlushTxFifo(&status_byte);
    print_cc1201_status(status_byte, "FLUSH_TX");
    CC1201_FlushRxFifo(&status_byte);
    print_cc1201_status(status_byte, "FLUSH_RX");
    
    // Test TX FIFO write operations
    printf("\n2. Testing TX FIFO Write Operations:\n\r");
    
    // Test pattern 1: Sequential bytes
    uint8_t test_data_1[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    status = CC1201_WriteTxFifo(test_data_1, sizeof(test_data_1), &status_byte);
    printf("  Write 5 bytes [01,02,03,04,05]: HAL=%d ", status);
    if (status == HAL_OK) {
        print_cc1201_status(status_byte, "TX_WRITE");
        
        // Check TX FIFO count
        uint8_t tx_count = 0;
        CC1201_GetNumTXBytes(&tx_count);
        printf("  TX FIFO count: %d bytes\n\r", tx_count);
    }
    
    // Test single byte write
    printf("\n3. Testing Single Byte Write:\n\r");
    status = CC1201_WriteSingleTxFifo(0xAA, &status_byte);
    printf("  Write single byte [AA]: HAL=%d ", status);
    if (status == HAL_OK) {
        print_cc1201_status(status_byte, "SINGLE_WRITE");
        
        uint8_t tx_count = 0;
        CC1201_GetNumTXBytes(&tx_count);
        printf("  TX FIFO count after single write: %d bytes\n\r", tx_count);
    }
    
    // Test RX FIFO read (should be empty)
    printf("\n4. Testing RX FIFO Read (should be empty):\n\r");
    uint8_t rx_buffer[10] = {0};
    status = CC1201_ReadRxFifo(rx_buffer, 1, &status_byte);
    printf("  Read 1 byte from RX FIFO: HAL=%d ", status);
    if (status == HAL_OK) {
        print_cc1201_status(status_byte, "RX_READ");
        printf("  Read data: 0x%02X\n\r", rx_buffer[0]);
        
        uint8_t rx_count = 0;
        CC1201_GetNumRXBytes(&rx_count);
        printf("  RX FIFO count: %d bytes\n\r", rx_count);
    }
    
    // Clean up - flush TX FIFO
    printf("\n5. Cleanup - Flushing TX FIFO:\n\r");
    CC1201_FlushTxFifo(&status_byte);
    print_cc1201_status(status_byte, "CLEANUP_FLUSH");
    
    uint8_t final_tx_count = 0;
    CC1201_GetNumTXBytes(&final_tx_count);
    printf("  Final TX FIFO count: %d bytes\n\r", final_tx_count);
    
    printf("=== BUFFER TEST COMPLETE ===\n\r");
}

// Test 2: State Change Operations
void test_state_changes(void) {
    printf("\n=== TEST 2: STATE CHANGE OPERATIONS ===\n\r");
    
    HAL_StatusTypeDef status;
    uint8_t status_byte = 0;
    uint8_t marc_state = 0;
    
    // Test 1: IDLE State
    printf("1. Testing IDLE State:\n\r");
    status = CC1201_EnterIdleMode(&status_byte);
    printf("  Enter IDLE: HAL=%d ", status);
    if (status == HAL_OK) {
        print_cc1201_status(status_byte, "IDLE");
        CC1201_ReadMARCState(&marc_state);
        printf("  MARC State: 0x%02X\n\r", marc_state);
    }
    HAL_Delay(50);
    
    // Test 2: RX State
    printf("\n2. Testing RX State:\n\r");
    status = CC1201_EnterRxMode(&status_byte);
    printf("  Enter RX: HAL=%d ", status);
    if (status == HAL_OK) {
        print_cc1201_status(status_byte, "RX");
        HAL_Delay(100); // Stay in RX for a moment
        CC1201_ReadMARCState(&marc_state);
        printf("  MARC State after delay: 0x%02X\n\r", marc_state);
    }
    
    // Test 3: Return to IDLE from RX
    printf("\n3. Testing IDLE from RX:\n\r");
    status = CC1201_EnterIdleMode(&status_byte);
    printf("  RX->IDLE: HAL=%d ", status);
    if (status == HAL_OK) {
        print_cc1201_status(status_byte, "RX_TO_IDLE");
    }
    HAL_Delay(50);
    
    // Test 4: TX State
    printf("\n4. Testing TX State:\n\r");
    status = CC1201_EnterTxMode(&status_byte);
    printf("  Enter TX: HAL=%d ", status);
    if (status == HAL_OK) {
        print_cc1201_status(status_byte, "TX");
        HAL_Delay(50); // Brief TX state
        CC1201_ReadMARCState(&marc_state);
        printf("  MARC State in TX: 0x%02X\n\r", marc_state);
    }
    
    // Test 5: Return to IDLE from TX
    printf("\n5. Testing IDLE from TX:\n\r");
    status = CC1201_EnterIdleMode(&status_byte);
    printf("  TX->IDLE: HAL=%d ", status);
    if (status == HAL_OK) {
        print_cc1201_status(status_byte, "TX_TO_IDLE");
    }
    
    // Test 6: Fast TX On (FSTXON state)
    printf("\n6. Testing Fast TX On (FSTXON):\n\r");
    status = CC1201_FastTxOn(&status_byte);
    printf("  Fast TX On: HAL=%d ", status);
    if (status == HAL_OK) {
        print_cc1201_status(status_byte, "FSTXON");
        CC1201_ReadMARCState(&marc_state);
        printf("  MARC State in FSTXON: 0x%02X\n\r", marc_state);
    }
    
    // Return to IDLE
    printf("\n7. Final return to IDLE:\n\r");
    status = CC1201_EnterIdleMode(&status_byte);
    printf("  Final IDLE: HAL=%d ", status);
    if (status == HAL_OK) {
        print_cc1201_status(status_byte, "FINAL_IDLE");
    }
    
    printf("=== STATE CHANGE TEST COMPLETE ===\n\r");
}

// Test 3: Individual Function Tests
void test_individual_functions(void) {
    printf("\n=== TEST 3: INDIVIDUAL FUNCTION TESTS ===\n\r");
    
    HAL_StatusTypeDef status;
    uint8_t data = 0;
    uint8_t status_byte = 0;
    
    // Test CC1201_ReadStatus (using a safe register like IOCFG2)
    printf("1. Testing CC1201_ReadStatus():\n\r");
    status = CC1201_ReadStatus(0x01, &data); // IOCFG2 register
    printf("  Read IOCFG2 (0x01): HAL=%d, Data=0x%02X\n\r", status, data);
    if (status == HAL_OK) {
        printf("  ✓ CC1201_ReadStatus() WORKING\n\r");
    } else {
        printf("  ✗ CC1201_ReadStatus() FAILED\n\r");
    }
    
    // Test CC1201_WriteRegister and verify with read
    printf("\n2. Testing CC1201_WriteRegister():\n\r");
    uint8_t original_value = data; // Store original value
    uint8_t test_value = 0x55; // Test pattern
    
    status = CC1201_WriteRegister(0x01, test_value);
    printf("  Write IOCFG2 (0x55): HAL=%d\n\r", status);
    
    if (status == HAL_OK) {
        // Read back to verify
        status = CC1201_ReadStatus(0x01, &data);
        printf("  Read back: HAL=%d, Data=0x%02X\n\r", status, data);
        
        if (status == HAL_OK && data == test_value) {
            printf("  ✓ CC1201_WriteRegister() WORKING (Write/Read verified)\n\r");
        } else {
            printf("  ✗ CC1201_WriteRegister() FAILED (Data mismatch)\n\r");
        }
        
        // Restore original value
        CC1201_WriteRegister(0x01, original_value);
        printf("  Restored original value: 0x%02X\n\r", original_value);
    } else {
        printf("  ✗ CC1201_WriteRegister() FAILED\n\r");
    }
    
    // Test CC1201_SendStrobe (using NOP)
    printf("\n3. Testing CC1201_SendStrobe():\n\r");
    status = CC1201_SendStrobe(0x3D, &status_byte); // NOP strobe
    printf("  Send NOP strobe: HAL=%d ", status);
    if (status == HAL_OK) {
        print_cc1201_status(status_byte, "STROBE_NOP");
        printf("  ✓ CC1201_SendStrobe() WORKING\n\r");
    } else {
        printf("  ✗ CC1201_SendStrobe() FAILED\n\r");
    }
    
    // Test CC1201_ReadMARCState
    printf("\n4. Testing CC1201_ReadMARCState():\n\r");
    uint8_t marc_state = 0;
    status = CC1201_ReadMARCState(&marc_state);
    printf("  Read MARC State: HAL=%d, State=0x%02X\n\r", status, marc_state);
    if (status == HAL_OK) {
        printf("  ✓ CC1201_ReadMARCState() WORKING\n\r");
    } else {
        printf("  ✗ CC1201_ReadMARCState() FAILED\n\r");
    }
    
    // Test CC1201_GetNumRXBytes
    printf("\n5. Testing CC1201_GetNumRXBytes():\n\r");
    uint8_t rx_bytes = 0;
    status = CC1201_GetNumRXBytes(&rx_bytes);
    printf("  Get RX Bytes: HAL=%d, Count=%d\n\r", status, rx_bytes);
    if (status == HAL_OK) {
        printf("  ✓ CC1201_GetNumRXBytes() WORKING\n\r");
    } else {
        printf("  ✗ CC1201_GetNumRXBytes() FAILED\n\r");
    }
    
    // Test CC1201_GetNumTXBytes
    printf("\n6. Testing CC1201_GetNumTXBytes():\n\r");
    uint8_t tx_bytes = 0;
    status = CC1201_GetNumTXBytes(&tx_bytes);
    printf("  Get TX Bytes: HAL=%d, Count=%d\n\r", status, tx_bytes);
    if (status == HAL_OK) {
        printf("  ✓ CC1201_GetNumTXBytes() WORKING\n\r");
    } else {
        printf("  ✗ CC1201_GetNumTXBytes() FAILED\n\r");
    }
    
    printf("=== INDIVIDUAL FUNCTION TEST COMPLETE ===\n\r");
}

// Main test runner
void run_comprehensive_cc1201_tests(void) {
    printf("\n STARTING COMPREHENSIVE CC1201 TEST SUITE \n\r");
    printf("================================================\n\r");
    
    // Run all tests
    test_buffer_operations();
    HAL_Delay(500);
    
    test_state_changes();
    HAL_Delay(500);
    
    test_individual_functions();
    HAL_Delay(500);
    
    printf("\n COMPREHENSIVE TEST SUITE COMPLETE! \n\r");
    printf("=============================================\n\r");
    
    // Final status check
    uint8_t final_status = 0;
    CC1201_Nop(&final_status);
    print_cc1201_status(final_status, "FINAL_STATUS");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */
  
  // Basic hardware initialization completed

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN BSP */

  /* -- Sample board code to send message over COM1 port ---- */
  printf("Welcome to STM32 world !\n\r");

  /* -- Sample board code to switch on leds ---- */
  BSP_LED_On(LED_GREEN);
  
  // Now that UART is initialized, start CC1201 testing
  printf("\n=== CC1201 COMMUNICATION SYSTEM STARTUP ===\n\r");

  uint8_t test_status = 0;
  HAL_StatusTypeDef nop_result = CC1201_Nop(&test_status);
  
  printf("NOP returned - HAL: %d, Status: 0x%02X\n\r", nop_result, test_status);
  
  if (nop_result == HAL_OK) {
      printf("Basic CC1201 communication working!\n\r");
      BSP_LED_Off(LED_RED);
      BSP_LED_On(LED_GREEN);
  } else {
      printf("CC1201 communication failed!\n\r");
      BSP_LED_Off(LED_GREEN);
      BSP_LED_On(LED_RED);
  }
  
  // Try proper power-up sequence first
  HAL_Delay(1000);
  cc1201_power_up_sequence();
  
  // Run comprehensive hardware diagnostic
  HAL_Delay(500);
  comprehensive_cc1201_diagnostic();
  
  // Only run functional tests if basic communication works
  if (nop_result == HAL_OK && test_status != 0xFF) {
      HAL_Delay(1000);
      run_comprehensive_cc1201_tests();
  } else {
      printf("\nSkipping functional tests due to communication issues.\n\r");
      printf("Please resolve hardware issues first.\n\r");
  }

  /* USER CODE END BSP */
    /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PD5 (CC1201 Interrupt) */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD4 (CC1201 Reset) */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  // Set reset pin HIGH (inactive) by default
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_5)
    {
        // Handle CC1201 interrupt
        printf("CC1201 Interrupt Triggered!\n\r");
        
        // Read MARC state to see what caused the interrupt
        uint8_t marc_state = 0;
        if (CC1201_ReadMARCState(&marc_state) == HAL_OK) {
            printf("MARC State during interrupt: 0x%02X\n\r", marc_state);
        }
        
        // Check if there are bytes in RX FIFO
        uint8_t rx_bytes = 0;
        if (CC1201_GetNumRXBytes(&rx_bytes) == HAL_OK) {
            printf("RX Bytes available: %d\n\r", rx_bytes);
            if (rx_bytes > 0) {
                // Could read FIFO data here if needed
                printf("Data received in RX FIFO!\n\r");
            }
        }
        
        // Toggle LED to indicate interrupt
        BSP_LED_Toggle(LED_GREEN);
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief BSP Push Button callback
  * @param Button Specifies the pressed button
  * @retval None
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    BspButtonState = BUTTON_PRESSED;
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
