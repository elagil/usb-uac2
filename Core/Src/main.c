/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "tusb.h"
#include "tas2780.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USBD_STACK_SIZE 4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

osThreadId_t usbTaskHandle;
const osThreadAttr_t usbTask_attributes = {
  .name = "usbTask",
  .stack_size = USBD_STACK_SIZE,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t amplifierTaskHandle;
const osThreadAttr_t amplifierTask_attributes = {
  .name = "amplifierTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void usbTask(void *argument);
void amplifierTask(void *argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  // defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  amplifierTaskHandle = osThreadNew(amplifierTask, NULL, &amplifierTask_attributes);
  usbTaskHandle = osThreadNew(usbTask, NULL, &usbTask_attributes);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/**
* @}
*/
/**
* @}
*/

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 125;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void setup_amplifier(uint16_t address, uint8_t channel_selection)
{
  uint8_t buf[2];
  const uint32_t DEFAULT_TIMEOUT = 10000;

  buf[0] = TAS2780_PAGE_REG;
  buf[1] = 0x00u; // Page 0
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);

  buf[0] = TAS2780_BOOK_REG;
  buf[1] = 0x00u; // Go to book 0x00
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);

  buf[0] = TAS2780_SW_RESET_REG;
  buf[1] = (0x01u << TAS2780_SW_RESET_SW_RESET_POS) & TAS2780_SW_RESET_SW_RESET_MASK; // Perform software reset
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);

  osDelay(1);

  buf[0] = TAS2780_PAGE_REG;
  buf[1] = 0x01u; // Page 1
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);

  // undocumented
  buf[0] = 0x17u;
  buf[1] = 0xC0u;
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);

  buf[0] = TAS2780_LSR_REG;
  buf[1] = (0x01u << TAS2780_LSR_EN_LLSR_POS) & TAS2780_LSR_EN_LLSR_MASK;
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);

  // undocumented
  buf[0] = 0x21u;
  buf[1] = 0x00u;
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);

  // undocumented
  buf[0] = 0x35u;
  buf[1] = 0x74u;
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);

  buf[0] = TAS2780_PAGE_REG;
  buf[1] = 0xFDu;
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);

  // undocumented
  buf[0] = 0x0Du;
  buf[1] = 0x0Du;
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);

  // undocumented
  buf[0] = 0x3Eu;
  buf[1] = 0x4Au;
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);

  // undocumented
  buf[0] = 0x0Du;
  buf[1] = 0x00u;
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);

  buf[0] = TAS2780_PAGE_REG;
  buf[1] = 0x00u; // Page 0
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);

  buf[0] = TAS2780_CHNL_0_REG;
  buf[1] = ((0x03u << TAS2780_CHNL_0_CDS_MODE_POS) & TAS2780_CHNL_0_CDS_MODE_MASK) | ((TAS2780_CHNL_0_AMP_LEVEL_DEFAULT << TAS2780_CHNL_0_AMP_LEVEL_POS) & TAS2780_CHNL_0_AMP_LEVEL_MASK);
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);

  buf[0] = TAS2780_DC_BLK0_REG;
  buf[1] = ((0x01u << TAS2780_DC_BLK0_VBAT1S_MODE_POS) & TAS2780_DC_BLK0_VBAT1S_MODE_MASK) | ((0x01u << TAS2780_DC_BLK0_AMP_SS_POS) & TAS2780_DC_BLK0_AMP_SS_MASK) | ((0x01u << TAS2780_DC_BLK0_HPF_FREQ_PB_POS) & TAS2780_DC_BLK0_HPF_FREQ_PB_MASK);
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);

  buf[0] = TAS2780_PVDD_UVLO_REG;
  buf[1] = 0x0Eu; // 6.5 V
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);

  buf[0] = TAS2780_TDM_CFG2_REG;
  buf[1] = ((0x02u << TAS2780_TDM_CFG2_RX_SLEN_POS) & TAS2780_TDM_CFG2_RX_SLEN_MASK) | ((0x02u << TAS2780_TDM_CFG2_RX_WLEN_POS) & TAS2780_TDM_CFG2_RX_WLEN_MASK) | ((channel_selection << TAS2780_TDM_CFG2_RX_SCFG_POS) & TAS2780_TDM_CFG2_RX_SCFG_MASK);
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);

  buf[0] = TAS2780_MODE_CTRL_REG;
  buf[1] = ((0x00 << TAS2780_MODE_CTRL_MODE_POS) & TAS2780_MODE_CTRL_MODE_MASK);
  HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, DEFAULT_TIMEOUT);
}

bool tas2780_is_active(uint16_t address)
{
  uint8_t reg;
  HAL_I2C_Mem_Read(&hi2c1, address, TAS2780_MODE_CTRL_REG, I2C_MEMADD_SIZE_8BIT, &reg, 1, 10000);
  uint8_t state = (reg & TAS2780_MODE_CTRL_MODE_MASK) >> TAS2780_MODE_CTRL_MODE_POS;

  return ((state == TAS2780_MODE_CTRL_MODE_ACTIVE_WITHOUT_MUTE) || (state == TAS2780_MODE_CTRL_MODE_ACTIVE_WITH_MUTE));
}

/**
 * @brief  USB worker
 * @param  argument: Not used
 * @retval None
 */
void usbTask(void *argument)
{
  tusb_init();

  while (true)
  {
    tud_task();
  }
}

/**
 * @brief  Amplifier handling task
 * @param  argument: Not used
 * @retval None
 */
void amplifierTask(void *argument)
{
  while (true)
  {
    osDelay(1000);

    if (
        !tas2780_is_active(TAS2780_DEVICE_ADDRESS_A) ||
        !tas2780_is_active(TAS2780_DEVICE_ADDRESS_B) ||
        !tas2780_is_active(TAS2780_DEVICE_ADDRESS_C) ||
        !tas2780_is_active(TAS2780_DEVICE_ADDRESS_D))
    {
      setup_amplifier(TAS2780_DEVICE_ADDRESS_A, TAS2780_TDM_CFG2_RX_SCFG_MONO_LEFT);
      setup_amplifier(TAS2780_DEVICE_ADDRESS_B, TAS2780_TDM_CFG2_RX_SCFG_MONO_RIGHT);
      setup_amplifier(TAS2780_DEVICE_ADDRESS_C, TAS2780_TDM_CFG2_RX_SCFG_MONO_LEFT);
      setup_amplifier(TAS2780_DEVICE_ADDRESS_D, TAS2780_TDM_CFG2_RX_SCFG_MONO_RIGHT);
    }
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for (;;)
  {
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
