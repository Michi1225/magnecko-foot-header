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
#include "i2c.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FootController.h"

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
_Objects Obj;
FootController controller = FootController();


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/**
 * This function is called when to get input values
 */
void cb_get_inputs()
{
  uint16_t control_word = Obj.Control_Word;
  // //Bit 4 is the Magnetization request 
  // controller.requested_magnetization = control_word & (1 << 4);
  // //Bit 5 is the Demagnetization request
  // controller.requested_demagnetization = control_word & (1 << 5);
  // //Bit 6 is the Discharge request
  // controller.requested_discharge = control_word & (1 << 6);
}

/**
* This function is called when to set outputs values
 */
void cb_set_outputs()
{
  
  // Obj.Status_word = controller.fsm_.getStatusWord();
  Obj.Temperatures[0] = controller.imu.accel_data.axis_x;
  Obj.Temperatures[1] = controller.imu.accel_data.axis_y;
  Obj.Temperatures[2] = controller.imu.accel_data.axis_z;

  // Obj.Quaternions[0] = controller.rotation_data.quaternion_i;
  // Obj.Quaternions[1] = controller.rotation_data.quaternion_j;
  // Obj.Quaternions[2] = controller.rotation_data.quaternion_k;
  // Obj.Quaternions[3] = controller.rotation_data.quaternion_real;

  // Obj.Force_estimation = controller.force_estimation;
  // Obj.Contact_estimation = controller.contact_estimation;

  // Obj.Flags = controller.active_magnetization | 
  //             (controller.status_magnetization << 1);
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI6_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  while(HAL_GPIO_ReadPin(EEPROM_LOADED_GPIO_Port, EEPROM_LOADED_Pin) == GPIO_PIN_RESET){}
  controller.init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) Error_Handler(); //Start Control Timer
  HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_SET);
  while (1)
  {
    controller.runCommunication();
    if(HAL_GPIO_ReadPin(CHARGE_DONE_GPIO_Port, CHARGE_DONE_Pin) == GPIO_PIN_SET)
    {
      HAL_GPIO_WritePin(STATUS0_GPIO_Port, STATUS0_Pin, GPIO_PIN_RESET);
      if(HAL_GPIO_ReadPin(CHARGE_START_GPIO_Port, CHARGE_START_Pin) == GPIO_PIN_SET)
      {
        if(!controller.active_magnetization)
        {
          // HAL_GPIO_TogglePin(CHARGE_START_GPIO_Port, CHARGE_START_Pin);
          // HAL_Delay(0);
          // HAL_GPIO_TogglePin(CHARGE_START_GPIO_Port, CHARGE_START_Pin);
        }
      }
    }
    else
    {
      HAL_GPIO_WritePin(STATUS0_GPIO_Port, STATUS0_Pin, GPIO_PIN_SET);
    }
    HAL_Delay(0);
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
      HAL_TIM_Base_Start_IT(&htim6);

    }
    else //Rising edge
    {
      HAL_TIM_Base_Stop_IT(&htim6);
    }
  }
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1)
  {
    //end switching pereiod
    HAL_GPIO_WritePin(DRV_M_GPIO_Port, DRV_M_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV_P_GPIO_Port, DRV_P_Pin, GPIO_PIN_RESET);
    HAL_TIM_Base_Stop_IT(htim);
    controller.active_magnetization = false; //Reset active magnetization flag

    //Start charging Capacitors
    HAL_GPIO_WritePin(CHARGE_START_GPIO_Port, CHARGE_START_Pin, GPIO_PIN_SET);
  }
  else if (htim == &htim2)
  {
    // Update the controller
    controller.runControl();
  }
  else if (htim == &htim6)
  {
    HAL_TIM_Base_Stop_IT(htim);
    //Handle Button Press
    //Ignore button press if magnetization is active
    if(controller.active_magnetization /*|| HAL_GPIO_ReadPin(CHARGE_DONE_GPIO_Port, CHARGE_DONE_Pin)*/) 
    {
      return; //Do nothing if magnetization is active
    } 
    HAL_GPIO_TogglePin(STATUS1_GPIO_Port, STATUS1_Pin); //Set Status LED 1
    //Else, start charging capacitors
    HAL_GPIO_WritePin(CHARGE_START_GPIO_Port, CHARGE_START_Pin, GPIO_PIN_SET); //Set Charge Start Pin
    // HAL_GPIO_TogglePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin);
    //Wait for Capacitors to charge
    while(HAL_GPIO_ReadPin(CHARGE_DONE_GPIO_Port, CHARGE_DONE_Pin) == GPIO_PIN_SET){}
    HAL_GPIO_WritePin(STATUS0_GPIO_Port, STATUS0_Pin, GPIO_PIN_SET); //Set Status LED 0

    //Toggle magnetization with each button press
    controller.requested_demagnetization =   controller.status_magnetization;
    controller.requested_magnetization   =  !controller.status_magnetization;
    controller.magnetize(MAGNETIZATION_TIME);
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
  /* User can add his own implementation to report the HAL error return state */
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
