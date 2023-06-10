/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HALL_L_A HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)
#define HALL_L_B HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)
#define HALL_L_C HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)

#define HALL_R_A HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10)
#define HALL_R_B HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11)
#define HALL_R_C HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12)

#define L1_R_ON HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7, GPIO_PIN_RESET)
#define L2_R_ON HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0, GPIO_PIN_RESET)
#define L3_R_ON HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1, GPIO_PIN_RESET)

#define L1_R_OFF HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7, GPIO_PIN_SET)
#define L2_R_OFF HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0, GPIO_PIN_SET)
#define L3_R_OFF HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1, GPIO_PIN_SET)


#define L1_L_ON HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_RESET)
#define L2_L_ON HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_RESET)
#define L3_L_ON HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_RESET)

#define L1_L_OFF HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_SET)
#define L2_L_OFF HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_SET)
#define L3_L_OFF HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_SET)

#define H1_R_PWM_ON TIM8->CCR1=throttle_R
#define H2_R_PWM_ON TIM8->CCR2=throttle_R
#define H3_R_PWM_ON TIM8->CCR3=throttle_R

#define H1_R_PWM_OFF TIM8->CCR1=0
#define H2_R_PWM_OFF TIM8->CCR2=0
#define H3_R_PWM_OFF TIM8->CCR3=0
//
#define H1_L_PWM_ON TIM1->CCR1=throttle_L
#define H2_L_PWM_ON TIM1->CCR2=throttle_L
#define H3_L_PWM_ON TIM1->CCR3=throttle_L

//#define H1_L_PWM_ON HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1)
//#define H2_L_PWM_ON HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2)
//#define H3_L_PWM_ON HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3)

#define H1_L_PWM_OFF TIM1->CCR2=0
#define H2_L_PWM_OFF TIM1->CCR2=0
#define H3_L_PWM_OFF TIM1->CCR3=0

//#define H1_L_PWM_OFF HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1)
//#define H2_L_PWM_OFF HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2)
//#define H3_L_PWM_OFF HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3)

#define H1_L HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);


int throttle_L = 0;
int throttle_R = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
extern uint8_t UART2_rxBuffer[8];
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
    if (HALL_L_A == 1 && HALL_L_B == 0 && HALL_L_C == 1)    //STEP 1
    {
        L1_L_OFF;
        H1_L_PWM_ON;

        H2_L_PWM_OFF;
        L2_L_OFF;

        H3_L_PWM_OFF;
        L3_L_ON;
//        L1_L_OFF;
//        H1_L_PWM_ON;
//
//        H2_L_PWM_OFF;
//        L2_L_ON;
//
//        H3_L_PWM_OFF;
//        L3_L_OFF;

    } else if (HALL_L_A == 1 && HALL_L_B == 0 && HALL_L_C == 0)    //STEP 2
    {
        H1_L_PWM_OFF;
        L1_L_OFF;

        H2_L_PWM_ON;
        L2_L_OFF;

        H3_L_PWM_OFF;
        L3_L_ON;
//        H1_L_PWM_OFF;
//        L1_L_OFF;
//
//        H2_L_PWM_OFF;
//        L2_L_ON;
//
//        L3_L_OFF;
//        H3_L_PWM_ON;

    } else if (HALL_L_A == 1 && HALL_L_B == 1 && HALL_L_C == 0)    //STEP 3
    {
        H1_L_PWM_OFF;
        L1_L_ON;

        L2_L_OFF;
        H2_L_PWM_ON;

        H3_L_PWM_OFF;
        L3_L_OFF;
//        H1_L_PWM_OFF;
//        L1_L_ON;
//
//        H2_L_PWM_OFF;
//        L2_L_OFF;
//
//        L3_L_OFF;
//        H3_L_PWM_ON;

    } else if (HALL_L_A == 0 && HALL_L_B == 1 && HALL_L_C == 0)    //STEP 4
    {
        H1_L_PWM_OFF;
        L1_L_ON;

        H2_L_PWM_OFF;
        L2_L_OFF;

        L3_L_OFF;
        H3_L_PWM_ON;
//        H1_L_PWM_OFF;
//        L1_L_ON;
//
//        L2_L_OFF;
//        H2_L_PWM_ON;
//
//        H3_L_PWM_OFF;
//        L3_L_OFF;

    } else if (HALL_L_A == 0 && HALL_L_B == 1 && HALL_L_C == 1)    //STEP 5
    {
        H1_L_PWM_OFF;
        L1_L_OFF;

        H2_L_PWM_OFF;
        L2_L_ON;

        L3_L_OFF;
        H3_L_PWM_ON;
//        H1_L_PWM_OFF;
//        L1_L_OFF;
//
//        H2_L_PWM_ON;
//        L2_L_OFF;
//
//        H3_L_PWM_OFF;
//        L3_L_ON;

    } else if (HALL_L_A == 0 && HALL_L_B == 0 && HALL_L_C == 1)    //STEP 6
    {
        L1_L_OFF;
        H1_L_PWM_ON;

        H2_L_PWM_OFF;
        L2_L_ON;

        H3_L_PWM_OFF;
        L3_L_OFF;
//        L1_L_OFF;
//        H1_L_PWM_ON;
//
//        H2_L_PWM_OFF;
//        L2_L_OFF;
//
//        H3_L_PWM_OFF;
//        L3_L_ON;
    }

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt.
  */
void TIM8_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_IRQn 0 */
    if (HALL_R_A == 1 && HALL_R_B == 0 && HALL_R_C == 1)    //STEP 1
    {
        L1_R_OFF;
        H1_R_PWM_ON;

        H2_R_PWM_OFF;
        L2_R_OFF;

        H3_R_PWM_OFF;
        L3_R_ON;

    } else if (HALL_R_A == 1 && HALL_R_B == 0 && HALL_R_C == 0)    //STEP 2
    {
        H1_R_PWM_OFF;
        L1_R_OFF;

        H2_R_PWM_ON;
        L2_R_OFF;

        H3_R_PWM_OFF;
        L3_R_ON;

    } else if (HALL_R_A == 1 && HALL_R_B == 1 && HALL_R_C == 0)    //STEP 3
    {
        H1_R_PWM_OFF;
        L1_R_ON;

        L2_R_OFF;
        H2_R_PWM_ON;

        H3_R_PWM_OFF;
        L3_R_OFF;
    } else if (HALL_R_A == 0 && HALL_R_B == 1 && HALL_R_C == 0)    //STEP 4
    {
        H1_R_PWM_OFF;
        L1_R_ON;

        H2_R_PWM_OFF;
        L2_R_OFF;

        L3_R_OFF;
        H3_R_PWM_ON;

    } else if (HALL_R_A == 0 && HALL_R_B == 1 && HALL_R_C == 1)    //STEP 5
    {
        H1_R_PWM_OFF;
        L1_R_OFF;

        H2_R_PWM_OFF;
        L2_R_ON;

        L3_R_OFF;
        H3_R_PWM_ON;

    } else if (HALL_R_A == 0 && HALL_R_B == 0 && HALL_R_C == 1)    //STEP 6
    {
        L1_R_OFF;
        H1_R_PWM_ON;

        H2_R_PWM_OFF;
        L2_R_ON;

        H3_R_PWM_OFF;
        L3_R_OFF;
    }

  /* USER CODE END TIM8_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_UP_IRQn 1 */

  /* USER CODE END TIM8_UP_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2){
        HAL_UART_Transmit_DMA(&huart2, UART2_rxBuffer, sizeof(UART2_rxBuffer));
        throttle_L = throttle_R = atoi(UART2_rxBuffer);
        HAL_UART_Receive_DMA(&huart2, UART2_rxBuffer, sizeof(UART2_rxBuffer));
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
//    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

}
/* USER CODE END 1 */
