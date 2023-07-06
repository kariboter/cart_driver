/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define R_MTR_CURR_Pin GPIO_PIN_0
#define R_MTR_CURR_GPIO_Port GPIOC
#define L_MTR_CURR_Pin GPIO_PIN_1
#define L_MTR_CURR_GPIO_Port GPIOC
#define V_BATT_MEAS_Pin GPIO_PIN_2
#define V_BATT_MEAS_GPIO_Port GPIOC
#define R_MTR_PHB_SENSE_Pin GPIO_PIN_3
#define R_MTR_PHB_SENSE_GPIO_Port GPIOC
#define R_MTR_PHA_SENSE_Pin GPIO_PIN_0
#define R_MTR_PHA_SENSE_GPIO_Port GPIOA
#define AIN_V_BATT_Pin GPIO_PIN_1
#define AIN_V_BATT_GPIO_Port GPIOA
#define UART_L_TX_Pin GPIO_PIN_2
#define UART_L_TX_GPIO_Port GPIOA
#define UART_L_RX_Pin GPIO_PIN_3
#define UART_L_RX_GPIO_Port GPIOA
#define SPEAKER_Pin GPIO_PIN_4
#define SPEAKER_GPIO_Port GPIOA
#define SYPPLY_EN_MCU_Pin GPIO_PIN_5
#define SYPPLY_EN_MCU_GPIO_Port GPIOA
#define R_MTR_OVERCUR_Pin GPIO_PIN_6
#define R_MTR_OVERCUR_GPIO_Port GPIOA
#define R_MTR_PHA_LO_Pin GPIO_PIN_7
#define R_MTR_PHA_LO_GPIO_Port GPIOA
#define L_MTR_PHB_SENSE_Pin GPIO_PIN_4
#define L_MTR_PHB_SENSE_GPIO_Port GPIOC
#define L_MTR_PHC_SENSE_Pin GPIO_PIN_5
#define L_MTR_PHC_SENSE_GPIO_Port GPIOC
#define R_MTR_PHB_LO_Pin GPIO_PIN_0
#define R_MTR_PHB_LO_GPIO_Port GPIOB
#define R_MTR_PHC_LO_Pin GPIO_PIN_1
#define R_MTR_PHC_LO_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOB
#define UART_R_TX_Pin GPIO_PIN_10
#define UART_R_TX_GPIO_Port GPIOB
#define UART_R_RX_Pin GPIO_PIN_11
#define UART_R_RX_GPIO_Port GPIOB
#define L_MTR_OVERCUR_Pin GPIO_PIN_12
#define L_MTR_OVERCUR_GPIO_Port GPIOB
#define L_MTR_PHA_LO_Pin GPIO_PIN_13
#define L_MTR_PHA_LO_GPIO_Port GPIOB
#define L_MTR_PHB_LO_Pin GPIO_PIN_14
#define L_MTR_PHB_LO_GPIO_Port GPIOB
#define L_MTR_PHC_LO_Pin GPIO_PIN_15
#define L_MTR_PHC_LO_GPIO_Port GPIOB
#define R_MTR_PHA_HI_Pin GPIO_PIN_6
#define R_MTR_PHA_HI_GPIO_Port GPIOC
#define R_MTR_PHB_HI_Pin GPIO_PIN_7
#define R_MTR_PHB_HI_GPIO_Port GPIOC
#define R_MTR_PHC_HI_Pin GPIO_PIN_8
#define R_MTR_PHC_HI_GPIO_Port GPIOC
#define L_MTR_PHA_HI_Pin GPIO_PIN_8
#define L_MTR_PHA_HI_GPIO_Port GPIOA
#define L_MTR_PHB_HI_Pin GPIO_PIN_9
#define L_MTR_PHB_HI_GPIO_Port GPIOA
#define L_MTR_PHC_HI_Pin GPIO_PIN_10
#define L_MTR_PHC_HI_GPIO_Port GPIOA
#define CHARGE_CONNECTED_Pin GPIO_PIN_12
#define CHARGE_CONNECTED_GPIO_Port GPIOA
#define R_MTR_HALL_PHC_Pin GPIO_PIN_10
#define R_MTR_HALL_PHC_GPIO_Port GPIOC
#define R_MTR_HALL_PHB_Pin GPIO_PIN_11
#define R_MTR_HALL_PHB_GPIO_Port GPIOC
#define R_MTR_HALL_PHA_Pin GPIO_PIN_12
#define R_MTR_HALL_PHA_GPIO_Port GPIOC
#define L_MTR_HALL_PHA_Pin GPIO_PIN_5
#define L_MTR_HALL_PHA_GPIO_Port GPIOB
#define L_MTR_HALL_PHB_Pin GPIO_PIN_6
#define L_MTR_HALL_PHB_GPIO_Port GPIOB
#define L_MTR_HALL_PHC_Pin GPIO_PIN_7
#define L_MTR_HALL_PHC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
