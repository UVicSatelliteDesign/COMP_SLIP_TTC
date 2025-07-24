/* CC1201 Chip Detection Utility */

#include "main.h" 
#include "CC1201_simple_link_reg_config.h"
#include <stdio.h>

/**
 * @brief Simple CC1201 chip detection test
 * Tries various commands to see if chip responds with anything other than 0x00
 */
uint8_t CC1201_ChipDetectionTest(void) {
    printf("\n=== CC1201 CHIP DETECTION TEST ===\n\r");
    
    uint8_t detection_score = 0;
    uint8_t non_zero_responses = 0;
    uint8_t response_pattern[16];
    
    // Test 1: Try multiple NOP commands
    printf("1. Testing NOP command pattern:\n\r");
    for (int i = 0; i < 8; i++) {
        uint8_t status = 0;
        HAL_StatusTypeDef result = CC1201_SendStrobe(0x3D, &status); // NOP
        response_pattern[i] = status;
        printf("   NOP %d: 0x%02X", i+1, status);
        
        if (status != 0x00) {
            non_zero_responses++;
            printf(" ✓");
        }
        printf("\n\r");
        HAL_Delay(10);
    }
    
    // Test 2: Try different strobe commands
    printf("\n2. Testing different strobe commands:\n\r");
    uint8_t strobes[] = {0x30, 0x34, 0x36, 0x37, 0x38, 0x3A, 0x3B, 0x3C};
    const char* names[] = {"RESET", "IDLE", "CAL", "TX", "RX", "FLUSH_RX", "FLUSH_TX", "WOR_RST"};
    
    for (int i = 0; i < 8; i++) {
        uint8_t status = 0;
        HAL_StatusTypeDef result = CC1201_SendStrobe(strobes[i], &status);
        response_pattern[i + 8] = status;
        printf("   %s: 0x%02X", names[i], status);
        
        if (status != 0x00) {
            non_zero_responses++;
            printf(" ✓");
        }
        printf("\n\r");
        HAL_Delay(50); // Longer delay for state changes
    }
    
    // Test 3: Analysis
    printf("\n3. DETECTION ANALYSIS:\n\r");
    printf("   Non-zero responses: %d/16\n\r", non_zero_responses);
    
    // Check for patterns that suggest a working chip
    uint8_t all_same = 1;
    for (int i = 1; i < 16; i++) {
        if (response_pattern[i] != response_pattern[0]) {
            all_same = 0;
            break;
        }
    }
    
    if (all_same) {
        printf("   ⚠ All responses identical (0x%02X) - likely no chip response\n\r", response_pattern[0]);
        detection_score = 0;
    } else {
        printf("   ✓ Response variation detected - chip likely present\n\r");
        detection_score = 50 + (non_zero_responses * 3);
    }
    
    // Test 4: Look for specific CC1201 behavior
    printf("\n4. CC1201 SPECIFIC BEHAVIOR TEST:\n\r");
    
    // CC1201 should respond to RESET with a status change
    uint8_t status_before = 0, status_after = 0;
    CC1201_SendStrobe(0x3D, &status_before); // NOP to get current status
    HAL_Delay(10);
    CC1201_SendStrobe(0x30, &status_after);  // RESET
    HAL_Delay(100); // Wait for reset
    
    printf("   Status before reset: 0x%02X\n\r", status_before);
    printf("   Status after reset:  0x%02X\n\r", status_after);
    
    if (status_before != status_after && (status_before != 0x00 || status_after != 0x00)) {
        printf("   ✓ Status changed after reset - good sign\n\r");
        detection_score += 25;
    } else {
        printf("   ⚠ No status change after reset\n\r");
    }
    
    // Final detection verdict
    printf("\n5. DETECTION VERDICT:\n\r");
    printf("   Detection Score: %d/100\n\r", detection_score);
    
    if (detection_score >= 75) {
        printf("   ✅ CC1201 DETECTED - Chip responding normally\n\r");
    } else if (detection_score >= 25) {
        printf("   ⚠ PARTIAL DETECTION - Chip may be present but not fully functional\n\r");
    } else {
        printf("   ❌ NO DETECTION - CC1201 likely not present or not responding\n\r");
        printf("      Possible causes:\n\r");
        printf("      • CC1201 not connected to SPI bus\n\r");
        printf("      • Power supply issue (check 3.3V)\n\r");
        printf("      • MISO line disconnected\n\r");
        printf("      • CC1201 in reset or sleep state\n\r");
        printf("      • Wrong SPI pin mapping\n\r");
    }
    
    printf("=== CHIP DETECTION TEST COMPLETE ===\n\r");
    return detection_score;
}
