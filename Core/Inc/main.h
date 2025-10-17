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
#include "stm32g4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Encoder_A_Pin GPIO_PIN_0
#define Encoder_A_GPIO_Port GPIOA
#define Encoder_B_Pin GPIO_PIN_1
#define Encoder_B_GPIO_Port GPIOA
#define Encoder_Z_Pin GPIO_PIN_2
#define Encoder_Z_GPIO_Port GPIOA
#define Encoder_Z_EXTI_IRQn EXTI2_IRQn
#define I_AC_max_Pin GPIO_PIN_4
#define I_AC_max_GPIO_Port GPIOA
#define Resolver_Exc_Pin GPIO_PIN_5
#define Resolver_Exc_GPIO_Port GPIOA
#define Resolver_Sin_Pin GPIO_PIN_6
#define Resolver_Sin_GPIO_Port GPIOA
#define Resolver_Cos_Pin GPIO_PIN_7
#define Resolver_Cos_GPIO_Port GPIOA
#define T_Motor_Pin GPIO_PIN_4
#define T_Motor_GPIO_Port GPIOC
#define T_IGBT_Pin GPIO_PIN_5
#define T_IGBT_GPIO_Port GPIOC
#define T_Ambient_Pin GPIO_PIN_2
#define T_Ambient_GPIO_Port GPIOB
#define I_U_FB_Pin GPIO_PIN_11
#define I_U_FB_GPIO_Port GPIOF
#define I_V_FB_Pin GPIO_PIN_12
#define I_V_FB_GPIO_Port GPIOF
#define MCU_I_AC_RST_Pin GPIO_PIN_15
#define MCU_I_AC_RST_GPIO_Port GPIOF
#define I_DC_Hi_Pin GPIO_PIN_7
#define I_DC_Hi_GPIO_Port GPIOE
#define I_DC_Lo_Pin GPIO_PIN_8
#define I_DC_Lo_GPIO_Port GPIOE
#define I_U_Hi_Pin GPIO_PIN_9
#define I_U_Hi_GPIO_Port GPIOE
#define I_U_Lo_Pin GPIO_PIN_10
#define I_U_Lo_GPIO_Port GPIOE
#define I_V_Hi_Pin GPIO_PIN_11
#define I_V_Hi_GPIO_Port GPIOE
#define I_V_Lo_Pin GPIO_PIN_12
#define I_V_Lo_GPIO_Port GPIOE
#define V_DC_Pin GPIO_PIN_13
#define V_DC_GPIO_Port GPIOE
#define AIN_1_Pin GPIO_PIN_11
#define AIN_1_GPIO_Port GPIOB
#define AIN_2_Pin GPIO_PIN_12
#define AIN_2_GPIO_Port GPIOB
#define DIN_2_Pin GPIO_PIN_13
#define DIN_2_GPIO_Port GPIOB
#define DIN_1_Pin GPIO_PIN_14
#define DIN_1_GPIO_Port GPIOB
#define DOUT_2_Pin GPIO_PIN_15
#define DOUT_2_GPIO_Port GPIOB
#define DOUT_1_Pin GPIO_PIN_8
#define DOUT_1_GPIO_Port GPIOD
#define DOUT_2_FB_Pin GPIO_PIN_9
#define DOUT_2_FB_GPIO_Port GPIOD
#define DOUT_1_FB_Pin GPIO_PIN_10
#define DOUT_1_FB_GPIO_Port GPIOD
#define DOUT_2_EN_Pin GPIO_PIN_11
#define DOUT_2_EN_GPIO_Port GPIOD
#define DOUT_1_EN_Pin GPIO_PIN_12
#define DOUT_1_EN_GPIO_Port GPIOD
#define Debug_LED_Pin GPIO_PIN_13
#define Debug_LED_GPIO_Port GPIOD
#define SPI1_CS_Driver_Pin GPIO_PIN_1
#define SPI1_CS_Driver_GPIO_Port GPIOG
#define PWM_UL_Pin GPIO_PIN_10
#define PWM_UL_GPIO_Port GPIOC
#define PWM_VL_Pin GPIO_PIN_11
#define PWM_VL_GPIO_Port GPIOC
#define PWM_WL_Pin GPIO_PIN_12
#define PWM_WL_GPIO_Port GPIOC
#define PWM_BKIN2_Pin GPIO_PIN_1
#define PWM_BKIN2_GPIO_Port GPIOD
#define PWM_BKIN_Pin GPIO_PIN_2
#define PWM_BKIN_GPIO_Port GPIOD
#define PWM_UH_Pin GPIO_PIN_6
#define PWM_UH_GPIO_Port GPIOB
#define PWM_VH_Pin GPIO_PIN_8
#define PWM_VH_GPIO_Port GPIOB
#define PWM_WH_Pin GPIO_PIN_9
#define PWM_WH_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
