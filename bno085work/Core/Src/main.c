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
#include "cmsis_os.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "BNO085_SPI_Library.h"
#include "Hardware_Init.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI1_PORT       GPIOA
#define SPI1_SCK_PIN    GPIO_PIN_5
#define SPI1_MISO_PIN   GPIO_PIN_6
#define SPI1_MOSI_PIN   GPIO_PIN_7
#define SPI1_AF_MAPPING GPIO_AF5_SPI1

#define CSN_PORT_S1     CSN_S1_GPIO_Port
#define CSN_PIN_S1      CSN_S1_Pin
#define INTN_PORT_S1    GPIOC
#define INTN_PIN_S1     GPIO_PIN_8
#define RSTN_PORT_S1    RSTN_S1_GPIO_Port
#define RSTN_PIN_S1     RSTN_S1_Pin
#define WAKE_PORT_S1    WAKE_S1_GPIO_Port
#define WAKE_PIN_S1     WAKE_S1_Pin
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
sensor_meta sensor1 = {0}; // zero-initialize the sensor metadata struct

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void MX_DWT_Init(void);
const char* get_accuracy_string(uint8_t accuracy);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

// This is the required shim function for microsecond delays
void delay_Us(uint32_t micros) {
    uint32_t start = DWT->CYCCNT;
    // Calculate the number of CPU cycles for the desired delay
    uint32_t cycles = micros * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < cycles) {
        // Busy wait
    }
}

// helper function to convert the accuracy level to a readable string.
const char* get_accuracy_string(uint8_t accuracy) {
    switch(accuracy) {
        case 0: return "Unreliable";
        case 1: return "Low";
        case 2: return "Medium";
        case 3: return "High";
        default: return "Unknown";
    }
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
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  MX_DWT_Init(); // Initialize the cycle counter for delay_Us

  uint8_t startup_status = N_ERR;

  // 1. Register Sensor Pins
  register_Sensor(&sensor1, 1, CSN_PIN_S1, CSN_PORT_S1, INTN_PIN_S1, INTN_PORT_S1, RSTN_PIN_S1, RSTN_PORT_S1);

  // 2. Initialize the library's hardware functions.
  bno085_library_spi_config_struct spi_config = {
      .SPI_Instance = SPI1
  };
  init_HardwareBNO085(&hspi1, spi_config);


  // 3. Initialize the BNO085 Sensor
  printf("Initializing BNO085...\r\n");
  // Set WAKE pin high BEFORE resetting to ensure SPI mode is selected
  HAL_GPIO_WritePin(WAKE_PORT_S1, WAKE_PIN_S1, GPIO_PIN_SET);

  init_GPIO_IMU(&sensor1);
  hardreset_IMU(&sensor1);

  // After a hard reset, the BNO085 sends two ad packetes
  // wait and clear before proceeding
  HAL_Delay(50);

  // Clear the first advertisement packet
  startup_status &= clear_init_Message_IMU(&sensor1);
  if(startup_status != N_ERR) {
      printf("Error: BNO085 did not send first advertisement packet.\r\n");
      Error_Handler();
  }

  // Clear the second advertisement packet
  startup_status &= clear_init_Message_IMU(&sensor1);
    if(startup_status != N_ERR) {
      printf("Error: BNO085 did not send second advertisement packet.\r\n");
      Error_Handler();
  }
  printf("BNO085 advertisement packets cleared.\r\n");

  // Verify we can communicate with the sensor by requesting its product ID
  startup_status &= check_Connection_IMU(&sensor1);
  if(startup_status != N_ERR) {
      printf("Error: Failed to get Product ID from BNO085.\r\n");
      Error_Handler();
  }
  printf("BNO085 connection successful.\r\n");

  get_and_clear_Reset_Status(&sensor1);
  // 4. Enable Desired Sensor Report
  printf("Enabling Game Rotation Vector at 100Hz...\r\n");

  if (startup_status != N_ERR) {
      printf("Error: Could not enable Game Rotation Vector.\r\n");
      Error_Handler();
  }


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // REMOVE THIS CODE - it's now in the FreeRTOS task!
    // if (data_available(&sensor1)) {
    //     ...
    // }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void MX_DWT_Init(void) {
    // Enable the DWT and its cycle counter
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
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
#ifdef USE_FULL_ASSERT
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
