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
SPI_HandleTypeDef hspi4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function to test GPIO pin states
void test_GPIO_pins(void) {
    printf("=== GPIO Pin Test ===\n\r");
    
    // Test CS pin
    printf("CS Pin (PE4): ");
    GPIO_PinState cs_state = HAL_GPIO_ReadPin(CC1201_CS_PORT, CC1201_CS_PIN);
    printf("%s\n\r", cs_state == GPIO_PIN_SET ? "HIGH" : "LOW");
    
    // Toggle CS pin to test control
    printf("Toggling CS pin...\n\r");
    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_RESET);
    HAL_Delay(100);
    cs_state = HAL_GPIO_ReadPin(CC1201_CS_PORT, CC1201_CS_PIN);
    printf("CS Pin after LOW: %s\n\r", cs_state == GPIO_PIN_SET ? "HIGH" : "LOW");
    
    HAL_GPIO_WritePin(CC1201_CS_PORT, CC1201_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(100);
    cs_state = HAL_GPIO_ReadPin(CC1201_CS_PORT, CC1201_CS_PIN);
    printf("CS Pin after HIGH: %s\n\r", cs_state == GPIO_PIN_SET ? "HIGH" : "LOW");
    
    // Test INT pin
    printf("INT Pin (PD4): ");
    GPIO_PinState int_state = HAL_GPIO_ReadPin(CC1201_INT_PORT, CC1201_INT_PIN);
    printf("%s\n\r", int_state == GPIO_PIN_SET ? "HIGH" : "LOW");
    
    printf("===================\n\r");
}

// Function to initialize CC1201 with preferred settings
HAL_StatusTypeDef initialize_CC1201(void) {
    uint8_t status_byte = 0;
    HAL_StatusTypeDef hal_status;
    
    printf("Initializing CC1201...\n\r");
    
    // Step 1: Soft reset
    hal_status = CC1201_SoftReset(&status_byte);
    if (hal_status != HAL_OK) {
        printf("CC1201 Reset Failed!\n\r");
        return hal_status;
    }
    printf("CC1201 Reset OK - Status: 0x%02X\n\r", status_byte);
    
    HAL_Delay(100); // Wait for reset to complete
    
    // Step 2: Write preferred settings (basic registers only)
    const registerSetting_t* settings = CC1201_GetPreferredSettings();
    uint16_t num_settings = CC1201_GetNumPreferredSettings();
    
    printf("Writing %d configuration registers...\n\r", num_settings);
    hal_status = CC1201_WriteRegisterConfig(settings, num_settings);
    if (hal_status != HAL_OK) {
        printf("Configuration write failed!\n\r");
        return hal_status;
    }
    printf("Configuration complete!\n\r");
    
    // Step 3: Enter idle mode
    hal_status = CC1201_EnterIdleMode(&status_byte);
    if (hal_status != HAL_OK) {
        printf("Enter Idle Failed!\n\r");
        return hal_status;
    }
    printf("Enter Idle OK - Status: 0x%02X\n\r", status_byte);
    
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
  MX_SPI4_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  
  // Test GPIO pins first
  test_GPIO_pins();
  
  // Initialize and test CC1201 communication
  printf("Starting CC1201 initialization...\n\r");
  HAL_Delay(100); // Give CC1201 time to power up
  
  if (initialize_CC1201() == HAL_OK) {
      printf("CC1201 initialization successful!\n\r");
      BSP_LED_On(LED_GREEN);
  } else {
      printf("CC1201 initialization failed!\n\r");
      BSP_LED_On(LED_RED);
  }
  
  // Run initial strobe command tests
  test_CC1201_strobe_commands();

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

  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_test_time = 0;
  const uint32_t test_interval = 5000; // Test every 5 seconds
  
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
      printf("Button pressed - Running CC1201 test...\n\r");
      test_CC1201_strobe_commands();
    }
    
    // Periodic CC1201 communication test
    if (HAL_GetTick() - last_test_time > test_interval) {
        last_test_time = HAL_GetTick();
        
        // Simple NOP test to verify communication
        uint8_t status_byte = 0;
        
        // First check CS pin state
        GPIO_PinState cs_state = HAL_GPIO_ReadPin(CC1201_CS_PORT, CC1201_CS_PIN);
        printf("CS Pin State: %s\n\r", cs_state == GPIO_PIN_SET ? "HIGH" : "LOW");
        
        // Try the NOP command
        HAL_StatusTypeDef hal_status = CC1201_Nop(&status_byte);
        
        printf("Periodic test - HAL Status: %d, CC1201 Status: 0x%02X\n\r", hal_status, status_byte);
        
        if (hal_status == HAL_OK && status_byte != 0xFF && status_byte != 0x00) {
            BSP_LED_On(LED_GREEN);  // Communication OK
            BSP_LED_Off(LED_RED);
        } else {
            BSP_LED_Off(LED_GREEN);
            BSP_LED_On(LED_RED);   // Communication error
            
            // Additional diagnostics
            if (hal_status != HAL_OK) {
                printf("  HAL SPI Error - Check SPI configuration\n\r");
            }
            if (status_byte == 0xFF) {
                printf("  Status 0xFF - Check connections/power\n\r");
            }
            if (status_byte == 0x00) {
                printf("  Status 0x00 - Check CS pin/timing\n\r");
            }
        }
    }
    
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
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_HARD_OUTPUT;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

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
