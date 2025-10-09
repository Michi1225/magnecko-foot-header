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
#include "adc.h"
#include "bdma.h"
#include "dma.h"
#include "i2c.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FootController.h"
#include <deque>

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
FootController controller = FootController();
std::deque<float> mag_avg0;
std::deque<float> mag_avg1;
std::deque<float> mag_avg2;
std::deque<float> mag_avg3;
bool prev_mag = false;
bool prev_demag = false;

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
  if((Obj.Magnet_Command & 0x01) && !prev_mag)
  {
    controller.requested_magnetization = true; //Set requested magnetization state
  }
  if((Obj.Magnet_Command & 0x02) && !prev_demag)
  {
    controller.requested_demagnetization = true; //Set requested demagnetization state
  }
  if(controller.requested_magnetization && controller.requested_demagnetization)
  {
    //If both magnetization and demagnetization are requested, reset both states
    controller.requested_magnetization = false;
    controller.requested_demagnetization = false;
  }

  prev_mag = static_cast<bool>(Obj.Magnet_Command & 0x01);
  prev_demag = static_cast<bool>(Obj.Magnet_Command & 0x02);
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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_SPI6_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C3_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  controller.init();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) Error_Handler(); //Start Control Timer
  if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) Error_Handler(); //Start BNO Timer
  HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_SET);
  std::deque<float> mag_avg0;
  while (1)
  {
    int error = controller.tof.get_ranging_data();
    std::memcpy(Obj.ToF_Data, controller.tof.data, sizeof(controller.tof.data));

    // //Windowed average of the magnetometer values
    mag_avg0.push_back(controller.hall0.read_magnitude());
    if(mag_avg0.size() > 20) mag_avg0.pop_front();
    float mag_avg0_sum = 0.0f;
    for(auto &val : mag_avg0)
    {
      mag_avg0_sum += val;
    }
    Obj.Force_Estimate = mag_avg0_sum / mag_avg0.size();
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == IMU_INT_Pin) {
    controller.imu.msg_ready = true;
  }
  else if(GPIO_Pin == BUTTON_Pin)
  {
    if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET) //Falling edge
    {
      //Handle Button Press
      TIM6->CNT = 0; //Reset Timer
      HAL_GPIO_WritePin(CHARGE_START_GPIO_Port, CHARGE_START_Pin, GPIO_PIN_SET);
      HAL_TIM_Base_Start_IT(&htim6);

    }
    else //Rising edge
    {
      HAL_TIM_Base_Stop_IT(&htim6);
      // HAL_GPIO_WritePin(CHARGE_START_GPIO_Port, CHARGE_START_Pin, GPIO_PIN_RESET);
    }
  }
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
  {
    // Update the controller
    controller.runControl();
    controller.runCommunication();
    return;
  }
  else if (htim == &htim1)
  {
    //end switching period
    HAL_GPIO_WritePin(DRV_M_GPIO_Port, DRV_M_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV_P_GPIO_Port, DRV_P_Pin, GPIO_PIN_RESET);
    HAL_TIM_Base_Stop_IT(htim);
    controller.active_magnetization = false; //Reset active magnetization flag

    //Start charging Capacitors
    HAL_GPIO_WritePin(CHARGE_START_GPIO_Port, CHARGE_START_Pin, GPIO_PIN_SET);
  }

  else if (htim == &htim3)
  {
    //BNO update
    controller.imu.update(); //Update IMU data
    Obj.IMU_Data.Acc_X = q_to_float(controller.imu.lin_accel_data.axis_x, controller.imu.lin_accel_data.q_point);
    Obj.IMU_Data.Acc_Y = q_to_float(controller.imu.lin_accel_data.axis_y, controller.imu.lin_accel_data.q_point);
    Obj.IMU_Data.Acc_Z = q_to_float(controller.imu.lin_accel_data.axis_z, controller.imu.lin_accel_data.q_point);

    Obj.IMU_Data.Gyro_X = q_to_float(controller.imu.gyro_data.axis_x, controller.imu.gyro_data.q_point);
    Obj.IMU_Data.Gyro_Y = q_to_float(controller.imu.gyro_data.axis_y, controller.imu.gyro_data.q_point);
    Obj.IMU_Data.Gyro_Z = q_to_float(controller.imu.gyro_data.axis_z, controller.imu.gyro_data.q_point);

    Obj.IMU_Data.Quat_I = q_to_float(controller.imu.rot_data.quaternion_i, controller.imu.rot_data.q_point);
    Obj.IMU_Data.Quat_J = q_to_float(controller.imu.rot_data.quaternion_j, controller.imu.rot_data.q_point);
    Obj.IMU_Data.Quat_K = q_to_float(controller.imu.rot_data.quaternion_k, controller.imu.rot_data.q_point);
    Obj.IMU_Data.Quat_R = q_to_float(controller.imu.rot_data.quaternion_real, controller.imu.rot_data.q_point);
  }



  else if (htim == &htim6)
  {
    HAL_TIM_Base_Stop_IT(htim);
    // HAL_GPIO_WritePin(CHARGE_START_GPIO_Port, CHARGE_START_Pin, GPIO_PIN_RESET);
    //Handle Button Press
    //Ignore button press if magnetization is active
    if(controller.active_magnetization) 
    {
      return; //Do nothing if magnetization is active
    }
    HAL_GPIO_TogglePin(STATUS1_GPIO_Port, STATUS1_Pin); //Set Status LED 1
    // HAL_GPIO_WritePin(CHARGE_START_GPIO_Port, CHARGE_START_Pin, GPIO_PIN_SET); //Charge Capacitors
    //Wait for Capacitors to charge
    // uint8_t cnt = 0;
    // while(true)
    // {
    //   if(HAL_GPIO_ReadPin(CHARGE_DONE_GPIO_Port, CHARGE_DONE_Pin) == GPIO_PIN_RESET) //Check if Capacitors are charged
    //   {
    //     ++cnt; //Increment counter
    //     if(cnt >= 10) //Deglitch
    //     {
    //       controller.charge_done = true; //Set charge done flag
    //       HAL_GPIO_WritePin(STATUS0_GPIO_Port, STATUS0_Pin, GPIO_PIN_SET); //Set Status LED 0
    //       break; //Exit loop

    //     }
    //   }
    //   else
    //   {
    //     cnt = 0; //Reset counter

    //   }
    // }

    //Toggle magnetization with each button press
    // controller.requested_demagnetization =   controller.status_magnetization;
    // controller.requested_magnetization   =  !controller.status_magnetization;
    controller.requested_demagnetization =   true;
    controller.requested_magnetization   =  false;
    controller.magnetize(MAGNETIZATION_TIME);
    controller.requested_demagnetization = false; //Reset requested magnetization state
    controller.requested_magnetization = false; //Reset requested demagnetization state
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
  HAL_GPIO_WritePin(DRV_M_GPIO_Port, DRV_M_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DRV_P_GPIO_Port, DRV_P_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CHARGE_START_GPIO_Port, CHARGE_START_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_RESET); //Discharge Caps

  /* User can add his own implementation to report the HAL error return state */
  HAL_GPIO_WritePin(DRV_M_GPIO_Port, DRV_M_Pin, GPIO_PIN_RESET); //Disable Switching
  HAL_GPIO_WritePin(DRV_P_GPIO_Port, DRV_P_Pin, GPIO_PIN_RESET); //Disable Switching
  HAL_GPIO_WritePin(CHARGE_START_GPIO_Port, CHARGE_START_Pin, GPIO_PIN_RESET); //Disable Charging
  HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_RESET); //Discharge


  for(int i = 0; i <= 162; ++i)
  {
    if( i != SysTick_IRQn) NVIC_DisableIRQ((IRQn_Type)i); //Disable all interrupts except SysTick
  }
  HAL_GPIO_WritePin(STATUS3_GPIO_Port, STATUS3_Pin, GPIO_PIN_SET); //Set Error LED
  while (1)
  {
    // Stay in this loop to indicate an error
    // You can also add a blinking LED or other error handling here
    HAL_Delay(200); // Delay to prevent flooding the console
    HAL_GPIO_TogglePin(STATUS3_GPIO_Port, STATUS3_Pin); //Toggle Error LED
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
