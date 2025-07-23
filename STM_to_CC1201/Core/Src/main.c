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

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
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

// Comprehensive CC1201 strobe command test
void test_all_strobe_commands(void) {
    uint8_t status_byte = 0;
    HAL_StatusTypeDef hal_status;
    
    printf("\n=== COMPREHENSIVE STROBE COMMAND TEST ===\n\r");
    
    // Test 1: NOP Command (baseline)
    printf("1. NOP Command (baseline test)\n\r");
    hal_status = CC1201_Nop(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "NOP");
    } else {
        printf("  NOP FAILED - HAL Error: %d\n\r", hal_status);
        return;
    }
    
    // Test 2: Soft Reset
    printf("\n2. Soft Reset\n\r");
    hal_status = CC1201_SoftReset(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "RESET");
        HAL_Delay(100); // Wait for reset to complete
    } else {
        printf("  RESET FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Test 3: Enter Idle Mode
    printf("\n3. Enter Idle Mode\n\r");
    hal_status = CC1201_EnterIdleMode(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "IDLE");
    } else {
        printf("  IDLE FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Test 4: Calibrate Frequency Synthesizer
    printf("\n4. Calibrate Frequency Synthesizer\n\r");
    hal_status = CC1201_CalFreqSynth(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "CALIBRATE");
        HAL_Delay(50); // Wait for calibration
    } else {
        printf("  CALIBRATE FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Test 5: Fast TX On
    printf("\n5. Fast TX On\n\r");
    hal_status = CC1201_FastTxOn(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "FAST_TX_ON");
    } else {
        printf("  FAST_TX_ON FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Test 6: Enter TX Mode
    printf("\n6. Enter TX Mode\n\r");
    hal_status = CC1201_EnterTxMode(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "TX_MODE");
        HAL_Delay(10);
    } else {
        printf("  TX_MODE FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Return to idle before RX test
    hal_status = CC1201_EnterIdleMode(&status_byte);
    HAL_Delay(10);
    
    // Test 7: Enter RX Mode
    printf("\n7. Enter RX Mode\n\r");
    hal_status = CC1201_EnterRxMode(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "RX_MODE");
        HAL_Delay(10);
    } else {
        printf("  RX_MODE FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Test 8: Flush RX FIFO
    printf("\n8. Flush RX FIFO\n\r");
    hal_status = CC1201_FlushRxFifo(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "FLUSH_RX");
    } else {
        printf("  FLUSH_RX FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Test 9: Flush TX FIFO
    printf("\n9. Flush TX FIFO\n\r");
    hal_status = CC1201_FlushTxFifo(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "FLUSH_TX");
    } else {
        printf("  FLUSH_TX FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Test 10: Auto Frequency Compensation
    printf("\n10. Auto Frequency Compensation\n\r");
    hal_status = CC1201_AutoFreqComp(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "AFC");
    } else {
        printf("  AFC FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Test 11: Wake on Radio
    printf("\n11. Wake on Radio\n\r");
    hal_status = CC1201_WakeOnRadio(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "WOR");
    } else {
        printf("  WOR FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Test 12: WOR Reset
    printf("\n12. WOR Reset\n\r");
    hal_status = CC1201_WorReset(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "WOR_RESET");
    } else {
        printf("  WOR_RESET FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Test 13: Enter Sleep Mode
    printf("\n13. Enter Sleep Mode\n\r");
    hal_status = CC1201_EnterSleepMode(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "SLEEP");
        HAL_Delay(100);
    } else {
        printf("  SLEEP FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Test 14: Wake up from sleep with NOP
    printf("\n14. Wake up from Sleep with NOP\n\r");
    hal_status = CC1201_Nop(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "WAKE_UP");
    } else {
        printf("  WAKE_UP FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Return to idle state
    hal_status = CC1201_EnterIdleMode(&status_byte);
    print_cc1201_status(status_byte, "FINAL_IDLE");
    
    printf("=== STROBE COMMAND TEST COMPLETE ===\n\r");
}

// Test reading status registers
void test_status_registers(void) {
    uint8_t data = 0;
    HAL_StatusTypeDef hal_status;
    
    printf("\n=== STATUS REGISTER TEST ===\n\r");
    
    // Test MARC State
    printf("1. Reading MARC State\n\r");
    hal_status = CC1201_ReadMARCState(&data);
    if (hal_status == HAL_OK) {
        printf("  MARC State: 0x%02X\n\r", data);
    } else {
        printf("  MARC State read FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Test RX Bytes
    printf("2. Reading RX FIFO Bytes\n\r");
    hal_status = CC1201_GetNumRXBytes(&data);
    if (hal_status == HAL_OK) {
        printf("  RX FIFO Bytes: %d\n\r", data);
    } else {
        printf("  RX Bytes read FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Test TX Bytes
    printf("3. Reading TX FIFO Bytes\n\r");
    hal_status = CC1201_GetNumTXBytes(&data);
    if (hal_status == HAL_OK) {
        printf("  TX FIFO Bytes: %d\n\r", data);
    } else {
        printf("  TX Bytes read FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    printf("=== STATUS REGISTER TEST COMPLETE ===\n\r");
}

// Test register read/write operations
void test_register_operations(void) {
    HAL_StatusTypeDef hal_status;
    uint8_t write_data = 0x55; // Test pattern
    uint8_t read_data = 0;
    
    printf("\n=== REGISTER READ/WRITE TEST ===\n\r");
    
    // Test with a safe register (IOCFG2 - 0x01)
    printf("1. Testing Register Write/Read (IOCFG2)\n\r");
    
    // First read original value
    hal_status = CC1201_ReadStatus(0x01, &read_data);
    if (hal_status == HAL_OK) {
        printf("  Original IOCFG2 value: 0x%02X\n\r", read_data);
        uint8_t original_value = read_data;
        
        // Write test pattern
        hal_status = CC1201_WriteRegister(0x01, write_data);
        if (hal_status == HAL_OK) {
            printf("  Write 0x%02X to IOCFG2: OK\n\r", write_data);
            
            // Read back
            hal_status = CC1201_ReadStatus(0x01, &read_data);
            if (hal_status == HAL_OK) {
                printf("  Read back: 0x%02X\n\r", read_data);
                if (read_data == write_data) {
                    printf("  ✓ Register write/read SUCCESS!\n\r");
                } else {
                    printf("  ✗ Register write/read FAILED - Data mismatch\n\r");
                }
                
                // Restore original value
                CC1201_WriteRegister(0x01, original_value);
                printf("  Restored original value: 0x%02X\n\r", original_value);
            } else {
                printf("  Read back FAILED - HAL Error: %d\n\r", hal_status);
            }
        } else {
            printf("  Write FAILED - HAL Error: %d\n\r", hal_status);
        }
    } else {
        printf("  Initial read FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    printf("=== REGISTER TEST COMPLETE ===\n\r");
}

// Test configuration writing
void test_configuration_write(void) {
    HAL_StatusTypeDef hal_status;
    
    printf("\n=== CONFIGURATION WRITE TEST ===\n\r");
    
    const registerSetting_t* settings = CC1201_GetPreferredSettings();
    uint16_t num_settings = CC1201_GetNumPreferredSettings();
    
    printf("Writing %d configuration registers...\n\r", num_settings);
    
    hal_status = CC1201_WriteRegisterConfig(settings, num_settings);
    if (hal_status == HAL_OK) {
        printf("  ✓ Configuration write SUCCESS!\n\r");
        
        // Verify a few key registers
        uint8_t read_data;
        printf("Verifying key registers:\n\r");
        
        // Check SYNC3 (should be 0x55)
        hal_status = CC1201_ReadStatus(0x03, &read_data);
        if (hal_status == HAL_OK) {
            printf("  SYNC3: 0x%02X (expected: 0x55)\n\r", read_data);
        }
        
        // Check PKT_LEN (should be 0xFF)
        hal_status = CC1201_ReadStatus(0x25, &read_data);
        if (hal_status == HAL_OK) {
            printf("  PKT_LEN: 0x%02X (expected: 0xFF)\n\r", read_data);
        }
        
    } else {
        printf("  ✗ Configuration write FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    printf("=== CONFIGURATION TEST COMPLETE ===\n\r");
}

// Test FIFO operations (read/write buffer tests)
void test_fifo_operations(void) {
    HAL_StatusTypeDef hal_status;
    uint8_t status_byte = 0;
    
    printf("\n=== FIFO OPERATIONS TEST ===\n\r");
    
    // First ensure we're in IDLE state
    hal_status = CC1201_EnterIdleMode(&status_byte);
    if (hal_status != HAL_OK) {
        printf("  ✗ Failed to enter IDLE mode\n\r");
        return;
    }
    print_cc1201_status(status_byte, "IDLE_FOR_FIFO");
    
    // Test 1: Flush both FIFOs to start clean
    printf("1. Flushing FIFOs\n\r");
    hal_status = CC1201_FlushTxFifo(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "FLUSH_TX");
    }
    
    hal_status = CC1201_FlushRxFifo(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "FLUSH_RX");
    }
    
    // Test 2: Check initial FIFO byte counts
    printf("2. Initial FIFO status\n\r");
    uint8_t tx_bytes = 0, rx_bytes = 0;
    
    hal_status = CC1201_GetNumTXBytes(&tx_bytes);
    if (hal_status == HAL_OK) {
        printf("  TX FIFO bytes: %d\n\r", tx_bytes);
    }
    
    hal_status = CC1201_GetNumRXBytes(&rx_bytes);
    if (hal_status == HAL_OK) {
        printf("  RX FIFO bytes: %d\n\r", rx_bytes);
    }
    
    // Test 3: Write test data to TX FIFO
    printf("3. Writing test data to TX FIFO\n\r");
    uint8_t test_data[] = {0xAA, 0x55, 0xCC, 0x33, 0xFF, 0x00, 0x11, 0x22};
    uint8_t test_data_length = sizeof(test_data);
    
    hal_status = CC1201_WriteTxFifo(test_data, test_data_length, &status_byte);
    if (hal_status == HAL_OK) {
        printf("  ✓ Wrote %d bytes to TX FIFO\n\r", test_data_length);
        print_cc1201_status(status_byte, "TX_WRITE");
        
        // Check TX FIFO count after write
        hal_status = CC1201_GetNumTXBytes(&tx_bytes);
        if (hal_status == HAL_OK) {
            printf("  TX FIFO bytes after write: %d (expected: %d)\n\r", tx_bytes, test_data_length);
            if (tx_bytes == test_data_length) {
                printf("  ✓ TX FIFO count matches written data\n\r");
            } else {
                printf("  ✗ TX FIFO count mismatch\n\r");
            }
        }
    } else {
        printf("  ✗ TX FIFO write FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Test 4: Write single byte to TX FIFO
    printf("4. Writing single byte to TX FIFO\n\r");
    hal_status = CC1201_WriteSingleTxFifo(0x42, &status_byte);
    if (hal_status == HAL_OK) {
        printf("  ✓ Wrote single byte (0x42) to TX FIFO\n\r");
        print_cc1201_status(status_byte, "TX_SINGLE");
        
        // Check updated count
        hal_status = CC1201_GetNumTXBytes(&tx_bytes);
        if (hal_status == HAL_OK) {
            printf("  TX FIFO bytes after single write: %d\n\r", tx_bytes);
        }
    } else {
        printf("  ✗ Single TX write FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Test 5: Attempt to read from RX FIFO (should be empty)
    printf("5. Reading from empty RX FIFO\n\r");
    uint8_t read_buffer[10] = {0};
    hal_status = CC1201_ReadRxFifo(read_buffer, 1, &status_byte);
    if (hal_status == HAL_OK) {
        printf("  Read from RX FIFO: 0x%02X\n\r", read_buffer[0]);
        print_cc1201_status(status_byte, "RX_READ_EMPTY");
        
        hal_status = CC1201_GetNumRXBytes(&rx_bytes);
        if (hal_status == HAL_OK) {
            printf("  RX FIFO bytes after read: %d\n\r", rx_bytes);
        }
    } else {
        printf("  ✗ RX FIFO read FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    // Test 6: Flush TX FIFO and verify
    printf("6. Flushing TX FIFO and verifying\n\r");
    hal_status = CC1201_FlushTxFifo(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "FINAL_FLUSH_TX");
        
        hal_status = CC1201_GetNumTXBytes(&tx_bytes);
        if (hal_status == HAL_OK) {
            printf("  TX FIFO bytes after flush: %d (expected: 0)\n\r", tx_bytes);
            if (tx_bytes == 0) {
                printf("  ✓ TX FIFO successfully flushed\n\r");
            } else {
                printf("  ✗ TX FIFO flush incomplete\n\r");
            }
        }
    } else {
        printf("  ✗ TX FIFO flush FAILED - HAL Error: %d\n\r", hal_status);
    }
    
    printf("=== FIFO OPERATIONS TEST COMPLETE ===\n\r");
}

// Advanced power state testing
void test_power_states(void) {
    HAL_StatusTypeDef hal_status;
    uint8_t status_byte = 0;
    
    printf("\n=== POWER STATE TESTING ===\n\r");
    
    // Test 1: Start from IDLE
    printf("1. Starting from IDLE state\n\r");
    hal_status = CC1201_EnterIdleMode(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "START_IDLE");
    }
    
    // Test 2: Enter RX mode
    printf("2. Entering RX mode\n\r");
    hal_status = CC1201_EnterRxMode(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "RX_MODE");
        HAL_Delay(100); // Stay in RX for a moment
    }
    
    // Test 3: Return to IDLE from RX
    printf("3. Returning to IDLE from RX\n\r");
    hal_status = CC1201_EnterIdleMode(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "IDLE_FROM_RX");
    }
    
    // Test 4: Enter TX mode
    printf("4. Entering TX mode\n\r");
    hal_status = CC1201_EnterTxMode(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "TX_MODE");
        HAL_Delay(50); // Brief TX state
    }
    
    // Test 5: Return to IDLE from TX
    printf("5. Returning to IDLE from TX\n\r");
    hal_status = CC1201_EnterIdleMode(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "IDLE_FROM_TX");
    }
    
    // Test 6: Fast TX On (FSTXON state)
    printf("6. Fast TX On (FSTXON state)\n\r");
    hal_status = CC1201_FastTxOn(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "FSTXON");
        HAL_Delay(50);
    }
    
    // Test 7: Return to IDLE from FSTXON
    printf("7. Returning to IDLE from FSTXON\n\r");
    hal_status = CC1201_EnterIdleMode(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "IDLE_FROM_FSTXON");
    }
    
    // Test 8: Enter Sleep mode
    printf("8. Entering Sleep mode\n\r");
    hal_status = CC1201_EnterSleepMode(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "SLEEP");
        HAL_Delay(200); // Sleep for a moment
    }
    
    // Test 9: Wake up from Sleep with NOP
    printf("9. Waking up from Sleep\n\r");
    hal_status = CC1201_Nop(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "WAKE_FROM_SLEEP");
    }
    
    // Test 10: Final IDLE state
    printf("10. Final IDLE state\n\r");
    hal_status = CC1201_EnterIdleMode(&status_byte);
    if (hal_status == HAL_OK) {
        print_cc1201_status(status_byte, "FINAL_IDLE");
    }
    
    printf("=== POWER STATE TESTING COMPLETE ===\n\r");
}

// Comprehensive test sequence
void run_comprehensive_cc1201_test(void) {
    printf("\n🚀 STARTING COMPREHENSIVE CC1201 TEST SEQUENCE 🚀\n\r");
    printf("================================================\n\r");
    
    test_all_strobe_commands();
    HAL_Delay(500);
    
    test_status_registers();
    HAL_Delay(500);
    
    test_register_operations();
    HAL_Delay(500);
    
    test_configuration_write();
    HAL_Delay(500);
    
    test_fifo_operations();
    HAL_Delay(500);
    
    test_power_states();
    HAL_Delay(500);
    
    printf("\n🎉 COMPREHENSIVE TEST SEQUENCE COMPLETE! 🎉\n\r");
    printf("==============================================\n\r");
}

// Function to test GPIO pin states
void test_GPIO_pins(void) {
    printf("=== GPIO Pin Test ===\n\r");
    
    // Test CS pin more thoroughly
    printf("CS Pin (PE4) Detailed Test:\n\r");
    GPIO_PinState cs_state = HAL_GPIO_ReadPin(CC1201_CS_PORT, CC1201_CS_PIN);
    printf("  Initial state: %s\n\r", cs_state == GPIO_PIN_SET ? "HIGH" : "LOW");
    
    // Force CS LOW and check multiple times
    printf("  Forcing CS LOW...\n\r");
    for (int i = 0; i < 5; i++) {
        HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_RESET);
        HAL_Delay(10);
        cs_state = HAL_GPIO_ReadPin(CC1201_CS_PORT, CC1201_CS_PIN);
        printf("    Attempt %d: %s\n\r", i+1, cs_state == GPIO_PIN_SET ? "HIGH" : "LOW");
    }
    
    // Force CS HIGH and check multiple times
    printf("  Forcing CS HIGH...\n\r");
    for (int i = 0; i < 5; i++) {
        HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_SET);
        HAL_Delay(10);
        cs_state = HAL_GPIO_ReadPin(CC1201_CS_PORT, CC1201_CS_PIN);
        printf("    Attempt %d: %s\n\r", i+1, cs_state == GPIO_PIN_SET ? "HIGH" : "LOW");
    }
    
    // Test INT pin
    printf("INT Pin (PD4): ");
    GPIO_PinState int_state = HAL_GPIO_ReadPin(CC1201_INT_PORT, CC1201_INT_PIN);
    printf("%s\n\r", int_state == GPIO_PIN_SET ? "HIGH" : "LOW");
    
    // Check SPI state
    printf("SPI2 State: ");
    if (CC1201_SPI_HANDLE.State == HAL_SPI_STATE_READY) {
        printf("READY\n\r");
    } else if (CC1201_SPI_HANDLE.State == HAL_SPI_STATE_BUSY) {
        printf("BUSY\n\r");
    } else if (CC1201_SPI_HANDLE.State == HAL_SPI_STATE_BUSY_TX) {
        printf("BUSY_TX\n\r");
    } else if (CC1201_SPI_HANDLE.State == HAL_SPI_STATE_BUSY_RX) {
        printf("BUSY_RX\n\r");
    } else if (CC1201_SPI_HANDLE.State == HAL_SPI_STATE_BUSY_TX_RX) {
        printf("BUSY_TX_RX\n\r");
    } else {
        printf("ERROR/RESET (%d)\n\r", CC1201_SPI_HANDLE.State);
    }
    
    printf("===================\n\r");
}

// Function to initialize CC1201 with preferred settings
HAL_StatusTypeDef initialize_CC1201(void) {
    printf("[DEBUG] Entered initialize_CC1201() function\n\r");
    
    uint8_t status_byte = 0;
    HAL_StatusTypeDef hal_status;
    
    printf("[DEBUG] Variables initialized\n\r");
    printf("Initializing CC1201...\n\r");
    printf("[DEBUG] After first printf in initialize_CC1201\n\r");
    printf("  Step 1: About to call SoftReset...\n\r");
    printf("[DEBUG] About to call CC1201_SoftReset\n\r");
    
    // Step 1: Soft reset
    hal_status = CC1201_SoftReset(&status_byte);
    printf("[DEBUG] CC1201_SoftReset returned\n\r");
    printf("  Step 1: SoftReset returned - HAL Status: %d, Status Byte: 0x%02X\n\r", hal_status, status_byte);
    
    if (hal_status != HAL_OK) {
        printf("CC1201 Reset Failed! HAL Status: %d\n\r", hal_status);
        return hal_status;
    }
    printf("CC1201 Reset OK - Status: 0x%02X\n\r", status_byte);
    
    printf("  Step 2: Waiting 100ms after reset...\n\r");
    HAL_Delay(100); // Wait for reset to complete
    
    printf("  Step 3: Getting preferred settings...\n\r");
    // Step 2: Write preferred settings (basic registers only)
    const registerSetting_t* settings = CC1201_GetPreferredSettings();
    uint16_t num_settings = CC1201_GetNumPreferredSettings();
    
    printf("Writing %d configuration registers...\n\r", num_settings);
    hal_status = CC1201_WriteRegisterConfig(settings, num_settings);
    printf("  Step 3: WriteRegisterConfig returned - HAL Status: %d\n\r", hal_status);
    
    if (hal_status != HAL_OK) {
        printf("Configuration write failed! HAL Status: %d\n\r", hal_status);
        return hal_status;
    }
    printf("Configuration complete!\n\r");
    
    printf("  Step 4: Entering IDLE mode...\n\r");
    // Step 3: Enter idle mode
    hal_status = CC1201_EnterIdleMode(&status_byte);
    printf("  Step 4: EnterIdleMode returned - HAL Status: %d, Status Byte: 0x%02X\n\r", hal_status, status_byte);
    
    if (hal_status != HAL_OK) {
        printf("Enter Idle Failed! HAL Status: %d\n\r", hal_status);
        return hal_status;
    }
    printf("Enter Idle OK - Status: 0x%02X\n\r", status_byte);
    
    printf("CC1201 initialization completed successfully!\n\r");
    return HAL_OK;
}

// Function to test CC1201 strobe commands
void test_CC1201_strobe_commands(void) {
    uint8_t status_byte = 0;
    HAL_StatusTypeDef hal_status;
    
    printf("Starting CC1201 Strobe Command Tests...\n\r");
    
    // Test 1: NOP command (should always work)
    printf("Test 1: NOP Command\n\r");
    hal_status = CC1201_Nop(&status_byte);
    if (hal_status == HAL_OK) {
        printf("  NOP Success - Status: 0x%02X\n\r", status_byte);
    } else {
        printf("  NOP Failed - HAL Status: %d\n\r", hal_status);
    }
    
    // Test 2: Soft Reset
    printf("Test 2: Soft Reset\n\r");
    hal_status = CC1201_SoftReset(&status_byte);
    if (hal_status == HAL_OK) {
        printf("  Soft Reset Success - Status: 0x%02X\n\r", status_byte);
    } else {
        printf("  Soft Reset Failed - HAL Status: %d\n\r", hal_status);
    }
    
    HAL_Delay(100); // Wait after reset
    
    // Test 3: Enter Idle Mode
    printf("Test 3: Enter Idle Mode\n\r");
    hal_status = CC1201_EnterIdleMode(&status_byte);
    if (hal_status == HAL_OK) {
        printf("  Enter Idle Success - Status: 0x%02X\n\r", status_byte);
    } else {
        printf("  Enter Idle Failed - HAL Status: %d\n\r", hal_status);
    }
    
    // Test 4: Read MARC State
    printf("Test 4: Read MARC State\n\r");
    uint8_t marc_state = 0;
    hal_status = CC1201_ReadMARCState(&marc_state);
    if (hal_status == HAL_OK) {
        printf("  MARC State: 0x%02X\n\r", marc_state);
    } else {
        printf("  Read MARC State Failed - HAL Status: %d\n\r", hal_status);
    }
    
    // Test 5: Calibrate Frequency Synthesizer
    printf("Test 5: Calibrate Frequency Synthesizer\n\r");
    hal_status = CC1201_CalFreqSynth(&status_byte);
    if (hal_status == HAL_OK) {
        printf("  Cal Freq Synth Success - Status: 0x%02X\n\r", status_byte);
    } else {
        printf("  Cal Freq Synth Failed - HAL Status: %d\n\r", hal_status);
    }
    
    // Test 6: Enter RX Mode
    printf("Test 6: Enter RX Mode\n\r");
    hal_status = CC1201_EnterRxMode(&status_byte);
    if (hal_status == HAL_OK) {
        printf("  Enter RX Success - Status: 0x%02X\n\r", status_byte);
    } else {
        printf("  Enter RX Failed - HAL Status: %d\n\r", hal_status);
    }
    
    HAL_Delay(10);
    
    // Test 7: Read MARC State again
    printf("Test 7: Read MARC State (after RX)\n\r");
    hal_status = CC1201_ReadMARCState(&marc_state);
    if (hal_status == HAL_OK) {
        printf("  MARC State: 0x%02X\n\r", marc_state);
    } else {
        printf("  Read MARC State Failed - HAL Status: %d\n\r", hal_status);
    }
    
    // Test 8: Flush RX FIFO
    printf("Test 8: Flush RX FIFO\n\r");
    hal_status = CC1201_FlushRxFifo(&status_byte);
    if (hal_status == HAL_OK) {
        printf("  Flush RX FIFO Success - Status: 0x%02X\n\r", status_byte);
    } else {
        printf("  Flush RX FIFO Failed - HAL Status: %d\n\r", hal_status);
    }
    
    // Test 9: Check RX bytes
    printf("Test 9: Check RX Bytes\n\r");
    uint8_t rx_bytes = 0;
    hal_status = CC1201_GetNumRXBytes(&rx_bytes);
    if (hal_status == HAL_OK) {
        printf("  RX Bytes: %d\n\r", rx_bytes);
    } else {
        printf("  Get RX Bytes Failed - HAL Status: %d\n\r", hal_status);
    }
    
    printf("CC1201 Strobe Command Tests Complete!\n\r");
    printf("========================================\n\r");
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  
  // Basic hardware initialization completed
  // CC1201 testing will be done after UART is initialized

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
  BSP_LED_On(LED_YELLOW);
  BSP_LED_On(LED_RED);
  
  // Now that UART is initialized, start CC1201 testing
  printf("\n=== CC1201 COMMUNICATION SYSTEM STARTUP ===\n\r");
  
  // Test GPIO pins first
  test_GPIO_pins();
  
  // Simple test instead of full initialization
  printf("Starting simple CC1201 test...\n\r");
  HAL_Delay(100);
  
  // First, test SPI2 peripheral directly
  printf("Testing SPI2 peripheral directly...\n\r");
  uint8_t spi_test_tx = 0xAA;
  uint8_t spi_test_rx = 0x00;
  
  HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_RESET); // CS Low
  HAL_StatusTypeDef spi_test_result = HAL_SPI_TransmitReceive(&CC1201_SPI_HANDLE, &spi_test_tx, &spi_test_rx, 1, 100);
  HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_SET); // CS High
  
  printf("Direct SPI test - HAL: %d, TX: 0x%02X, RX: 0x%02X\n\r", spi_test_result, spi_test_tx, spi_test_rx);
  
  if (spi_test_result == HAL_OK) {
      printf("SPI2 peripheral working!\n\r");
  } else {
      printf("SPI2 peripheral failed! Error: %d\n\r", spi_test_result);
      if (spi_test_result == HAL_TIMEOUT) printf("  -> SPI TIMEOUT\n\r");
      if (spi_test_result == HAL_ERROR) printf("  -> SPI ERROR\n\r");
      if (spi_test_result == HAL_BUSY) printf("  -> SPI BUSY\n\r");
  }
  
  printf("Test 1: Creating status byte variable...\n\r");
  uint8_t test_status = 0;
  
  printf("Test 2: About to call CC1201_Nop...\n\r");
  HAL_StatusTypeDef nop_result = CC1201_Nop(&test_status);
  
  printf("Test 3: NOP returned - HAL: %d, Status: 0x%02X\n\r", nop_result, test_status);
  
  if (nop_result == HAL_OK) {
      printf("Basic CC1201 communication working!\n\r");
      BSP_LED_Off(LED_RED);
      BSP_LED_On(LED_GREEN);
  } else {
      printf("CC1201 communication failed!\n\r");
      BSP_LED_Off(LED_GREEN);
      BSP_LED_On(LED_RED);
  }

  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_test = 0;
  uint32_t test_counter = 0;
  
  printf("\n🚀 STARTING SIMPLE CC1201 TEST MODE 🚀\n\r");
  
  while (1)
  {

    /* -- Sample board code for User push-button in interrupt mode ---- */
    if (BspButtonState == BUTTON_PRESSED)
    {
      /* Update button state */
      BspButtonState = BUTTON_RELEASED;
      /* -- Sample board code to toggle leds ---- */
      BSP_LED_Toggle(LED_GREEN);
      BSP_LED_Toggle(LED_YELLOW);
      BSP_LED_Toggle(LED_RED);

      /* ..... Perform your action ..... */
      printf("\n[BUTTON] Manual test triggered...\n\r");
      uint8_t button_status = 0;
      HAL_StatusTypeDef button_result = CC1201_Nop(&button_status);
      printf("Button test - HAL: %d, Status: 0x%02X\n\r", button_result, button_status);
    }
    
    // Simple periodic test every 5 seconds
    if (HAL_GetTick() - last_test > 5000) {
      printf("[%lu] Test %lu: ", HAL_GetTick(), ++test_counter);
      
      uint8_t status_byte = 0;
      HAL_StatusTypeDef hal_status = CC1201_Nop(&status_byte);
      
      if (hal_status == HAL_OK) {
        printf("OK - Status: 0x%02X\n\r", status_byte);
        BSP_LED_On(LED_GREEN);
        BSP_LED_Off(LED_RED);
      } else {
        printf("FAILED - HAL: %d\n\r", hal_status);
        BSP_LED_Off(LED_GREEN);
        BSP_LED_On(LED_RED);
      }
      
      last_test = HAL_GetTick();
    }
    
    HAL_Delay(100);
    
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == CC1201_INT_PIN)
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
