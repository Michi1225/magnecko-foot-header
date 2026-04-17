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
#include "bdma.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FootController.h"
#include "utils.h"
#include <deque>
#include <stdio.h>
#include <cstring>

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

/* USER CODE BEGIN PV */
_Objects Obj = {};
FootController __attribute__((section(".RAM"))) controller = FootController();

std::deque<float> mag_avg0;
std::deque<float> mag_avg1;
std::deque<float> mag_avg2;
std::deque<float> mag_avg3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */



/**
 * This function is called when to get input values
 */
void cb_get_inputs()
{
  controller.fsm_.setControlWord(Obj.Control_Word);
  
  //Magnetize on GPIO0 rising edge
  if((Obj.Magnet_Command & 0x01) && !controller.prev_mag)
  {
    controller.requested_magnetization = true; //Set requested magnetization state
  }
  if((Obj.Magnet_Command & 0x02) && !controller.prev_demag)
  {
    controller.requested_demagnetization = true; //Set requested demagnetization state
  }
  if(controller.requested_magnetization && controller.requested_demagnetization)
  {
    //If both magnetization and demagnetization are requested, reset both states
    controller.requested_magnetization = false;
    controller.requested_demagnetization = false;
  }

  controller.prev_mag = static_cast<bool>(Obj.Magnet_Command & 0x01);
  controller.prev_demag = static_cast<bool>(Obj.Magnet_Command & 0x02);
}

/**
* This function is called when to set outputs values
 */
void cb_set_outputs()
{
  Obj.Status_Word = controller.fsm_.getStatusWord();


}

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_SPI6_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  controller.init();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  std::deque<float> force_average;
  SWD_Init();
  while (1)
  {
    if(controller.tof.data_valid)
    {
      // TODO: correctly interpret data
      std::memcpy(Obj.ToF_Data, (void *)(&controller.tof.data_frame), sizeof(Obj.ToF_Data));
    }

    // Windowed average of force estimation
    force_average.push_back(
      TMAG5273::force_estimation(
        controller.hall0.b_mag,
        controller.hall1.b_mag,
        controller.hall2.b_mag,
        controller.hall3.b_mag)
    );
    if(force_average.size() > 20) force_average.pop_front();
    float mag_force_average_sum = 0.0f;
    for(auto &val : force_average)
    {
      mag_force_average_sum += val;
    }
    Obj.Force_Estimate = mag_force_average_sum / force_average.size();


    //Handle LDC
    for(LDC1101 &ldc : controller.ldc)
    {
      if(ldc.data_valid)
      {
        // TODO: Process LDC data and store in Obj if needed
        uint16_t rp_data = ldc.rx_data.rp_data;
        uint16_t l_data = ldc.rx_data.l_data;
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
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // Handle IMU Interrupt
  // When the IMU interrupt pin is triggered, set the msg_ready flag to true to indicate that new IMU data is available and should be read within the next 10ms.
  if(GPIO_Pin == IMU_INT_Pin) {
    controller.imu.msg_ready = true;
  }

  // Handle Button Press and Release
  // On Button Press, start a timer to measure how long the button is held down. On Button Release, stop the timer.
  // If the button is held down for more than a certain threshold (e.g., 1 second), trigger a demagnetization event.
  else if(GPIO_Pin == BUTTON_Pin)
  {
    // Button Pressed (Falling edge)
    if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET)
    {
      
      TIM6->CNT = 0;
      HAL_TIM_Base_Start_IT(TIM_BUTTON);

    }
    // Button Released (Rising edge)
    else
    {
      HAL_TIM_Base_Stop_IT(TIM_BUTTON);
    }
  }

  // Handle ToF interrupt
  else if(GPIO_Pin == TOF_INT_Pin)
  {
    //TODO: read ToF data
    controller.tof.get_ranging_data();
  }
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Controller and Communication Loop
  // This loop runs at 1kHz
  if (htim == TIM_CONTROL)
  {
    controller.runControl();
    controller.runCommunication();
    controller.active_ldc->read_data(); // Read data from the currently active LDC
    return;
  }


  // IMU Update Loop
  // This loop runs at 400Hz
  else if (htim == TIM_IMU)
  {
    controller.imu.update(); //Update IMU data
  }


  // Manual Demagnetization control
  // This interrupt is triggered, if the button is pressed for more than 1 second.
  else if (htim == TIM_BUTTON)
  {
    HAL_TIM_Base_Stop_IT(htim);
    controller.requested_demagnetization =   true;
    controller.requested_magnetization   =  false;
    controller.magnetize(MAGNETIZATION_TIME);
    controller.requested_demagnetization = false;
    controller.requested_magnetization = false;
  }
  

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == LDC_SPI_HANDLE)
    {
        HAL_GPIO_WritePin(LDC0_NCS_GPIO_Port, LDC0_NCS_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LDC1_NCS_GPIO_Port, LDC1_NCS_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LDC2_NCS_GPIO_Port, LDC2_NCS_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LDC3_NCS_GPIO_Port, LDC3_NCS_Pin, GPIO_PIN_SET);
        controller.active_ldc->data_valid = true; // Set the data_valid flag for the currently active LDC
        controller.active_ldc = controller.active_ldc->next; // Move to the next LDC for the next measurement
    }
    else if (hspi == TOF_SPI_HANDLE)
    {
        controller.tof.data_valid = true;
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
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  errorHandler();
  while (1)
  {
    // Stay in this loop to indicate an error
    // You can also add a blinking LED or other error handling here
    HAL_Delay(200); // Delay to prevent flooding the console
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
