/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "utypes.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern _Objects Obj;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi6;
extern I2C_HandleTypeDef hi2c3;
extern TIM_HandleTypeDef htim1;
extern I2C_HandleTypeDef hi2c1;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DISCHARGE_Pin GPIO_PIN_0
#define DISCHARGE_GPIO_Port GPIOC
#define DRV1_Pin GPIO_PIN_0
#define DRV1_GPIO_Port GPIOA
#define DRV2_Pin GPIO_PIN_1
#define DRV2_GPIO_Port GPIOA
#define EEPROM_LOADED_Pin GPIO_PIN_2
#define EEPROM_LOADED_GPIO_Port GPIOA
#define ECAT_IRQ_Pin GPIO_PIN_3
#define ECAT_IRQ_GPIO_Port GPIOA
#define ECAT_NCS_Pin GPIO_PIN_4
#define ECAT_NCS_GPIO_Port GPIOA
#define ECAT_SCK_Pin GPIO_PIN_5
#define ECAT_SCK_GPIO_Port GPIOA
#define ECAT_MOSI_Pin GPIO_PIN_6
#define ECAT_MOSI_GPIO_Port GPIOA
#define ECAT_MISO_Pin GPIO_PIN_7
#define ECAT_MISO_GPIO_Port GPIOA
#define SYNC0_Pin GPIO_PIN_4
#define SYNC0_GPIO_Port GPIOC
#define SYNC1_Pin GPIO_PIN_5
#define SYNC1_GPIO_Port GPIOC
#define BUTTON_Pin GPIO_PIN_0
#define BUTTON_GPIO_Port GPIOB
#define BUTTON_EXTI_IRQn EXTI0_IRQn
#define nRST_ECAT_Pin GPIO_PIN_1
#define nRST_ECAT_GPIO_Port GPIOB
#define CHARGER_SCK_Pin GPIO_PIN_10
#define CHARGER_SCK_GPIO_Port GPIOB
#define CHARGER_NCS_Pin GPIO_PIN_12
#define CHARGER_NCS_GPIO_Port GPIOB
#define CHARGER_MISO_Pin GPIO_PIN_14
#define CHARGER_MISO_GPIO_Port GPIOB
#define CHARGER_MOSI_Pin GPIO_PIN_15
#define CHARGER_MOSI_GPIO_Port GPIOB
#define TOF_INT_Pin GPIO_PIN_6
#define TOF_INT_GPIO_Port GPIOC
#define TOF_I2C_RST_Pin GPIO_PIN_7
#define TOF_I2C_RST_GPIO_Port GPIOC
#define SENS_SDA_Pin GPIO_PIN_9
#define SENS_SDA_GPIO_Port GPIOC
#define SENS_SCL_Pin GPIO_PIN_8
#define SENS_SCL_GPIO_Port GPIOA
#define TOF_LP_Pin GPIO_PIN_9
#define TOF_LP_GPIO_Port GPIOA
#define LDC_INT_Pin GPIO_PIN_10
#define LDC_INT_GPIO_Port GPIOA
#define IMU_WAKE_Pin GPIO_PIN_12
#define IMU_WAKE_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define IMU_NCS_Pin GPIO_PIN_15
#define IMU_NCS_GPIO_Port GPIOA
#define SENS_LS_SDA_Pin GPIO_PIN_10
#define SENS_LS_SDA_GPIO_Port GPIOC
#define SENS_LS_SCL_Pin GPIO_PIN_11
#define SENS_LS_SCL_GPIO_Port GPIOC
#define IMU_SCK_Pin GPIO_PIN_12
#define IMU_SCK_GPIO_Port GPIOC
#define IMU_INT_Pin GPIO_PIN_2
#define IMU_INT_GPIO_Port GPIOD
#define IMU_INT_EXTI_IRQn EXTI2_IRQn
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define IMU_MISO_Pin GPIO_PIN_4
#define IMU_MISO_GPIO_Port GPIOB
#define IMU_MOSI_Pin GPIO_PIN_5
#define IMU_MOSI_GPIO_Port GPIOB
#define IMU_BOOTN_Pin GPIO_PIN_6
#define IMU_BOOTN_GPIO_Port GPIOB
#define IMU_NRST_Pin GPIO_PIN_7
#define IMU_NRST_GPIO_Port GPIOB
#define LED_SCL_Pin GPIO_PIN_8
#define LED_SCL_GPIO_Port GPIOB
#define LED_SDA_Pin GPIO_PIN_9
#define LED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
