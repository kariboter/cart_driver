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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HALL_A HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)
#define HALL_B HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)
#define HALL_C HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)

#define L1_ON HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_RESET)
#define L2_ON HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_RESET)
#define L3_ON HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_RESET)

#define L1_OFF HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_SET)
#define L2_OFF HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_SET)
#define L3_OFF HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_SET)

#define H1_PWM_ON TIM1->CCR1=throttle
#define H2_PWM_ON TIM1->CCR2=throttle
#define H3_PWM_ON TIM1->CCR3=throttle

#define H1_PWM_OFF TIM1->CCR1=0
#define H2_PWM_OFF TIM1->CCR2=0
#define H3_PWM_OFF TIM1->CCR3=0
int throttle = (13439/10);
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
extern TIM_HandleTypeDef htim1;
/* USER CODE BEGIN EV */

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
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
    if (HALL_A == 1 && HALL_B == 0 && HALL_C == 1)    //STEP 1
    {
        L1_OFF;
        H1_PWM_ON;


        H2_PWM_OFF;
        L2_OFF;

        H3_PWM_OFF;
        L3_ON;

    } else if (HALL_A == 1 && HALL_B == 0 && HALL_C == 0)    //STEP 2
    {
        H1_PWM_OFF;
        L1_OFF;

        H2_PWM_ON;
        L2_OFF;

        H3_PWM_OFF;
        L3_ON;
    } else if (HALL_A == 1 && HALL_B == 1 && HALL_C == 0)    //STEP 3
    {
        H1_PWM_OFF;
        L1_ON;

        L2_OFF;
        H2_PWM_ON;

        H3_PWM_OFF;
        L3_OFF;
    } else if (HALL_A == 0 && HALL_B == 1 && HALL_C == 0)    //STEP 4
    {
        H1_PWM_OFF;
        L1_ON;

        H2_PWM_OFF;
        L2_OFF;

        L3_OFF;
        H3_PWM_ON;

    } else if (HALL_A == 0 && HALL_B == 1 && HALL_C == 1)    //STEP 5
    {
        H1_PWM_OFF;
        L1_OFF;

        H2_PWM_OFF;
        L2_ON;

        L3_OFF;
        H3_PWM_ON;

    } else if (HALL_A == 0 && HALL_B == 0 && HALL_C == 1)    //STEP 6
    {

        L1_OFF;
        H1_PWM_ON;


        H2_PWM_OFF;
        L2_ON;

        H3_PWM_OFF;
        L3_OFF;
    }


  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
