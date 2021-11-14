/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal.h"

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
#define DS3231_SQW_Pin GPIO_PIN_2
#define DS3231_SQW_GPIO_Port GPIOE
#define DS3231_SQW_EXTI_IRQn EXTI2_IRQn
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define D1_Pin GPIO_PIN_4
#define D1_GPIO_Port GPIOE
#define D2_Pin GPIO_PIN_5
#define D2_GPIO_Port GPIOE
#define Relay_9_Pin GPIO_PIN_6
#define Relay_9_GPIO_Port GPIOE
#define LED_ON_Pin GPIO_PIN_13
#define LED_ON_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define ADC_D10_Pin GPIO_PIN_1
#define ADC_D10_GPIO_Port GPIOC
#define ADC_INPUT6_Pin GPIO_PIN_2
#define ADC_INPUT6_GPIO_Port GPIOC
#define ADC_INPUT13_Pin GPIO_PIN_3
#define ADC_INPUT13_GPIO_Port GPIOC
#define ON_Pin GPIO_PIN_0
#define ON_GPIO_Port GPIOA
#define ON_EXTI_IRQn EXTI0_IRQn
#define ADC_D3_Pin GPIO_PIN_1
#define ADC_D3_GPIO_Port GPIOA
#define ADC_D5_Pin GPIO_PIN_4
#define ADC_D5_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define ADC_INPUT14_Pin GPIO_PIN_4
#define ADC_INPUT14_GPIO_Port GPIOC
#define ADC_INPUT15_Pin GPIO_PIN_5
#define ADC_INPUT15_GPIO_Port GPIOC
#define ADC_D6_Pin GPIO_PIN_0
#define ADC_D6_GPIO_Port GPIOB
#define ADC_D7_Pin GPIO_PIN_1
#define ADC_D7_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define Relay_0_Pin GPIO_PIN_7
#define Relay_0_GPIO_Port GPIOE
#define Relay_1_Pin GPIO_PIN_8
#define Relay_1_GPIO_Port GPIOE
#define Relay_2_Pin GPIO_PIN_9
#define Relay_2_GPIO_Port GPIOE
#define Relay_3_Pin GPIO_PIN_10
#define Relay_3_GPIO_Port GPIOE
#define Relay_4_Pin GPIO_PIN_11
#define Relay_4_GPIO_Port GPIOE
#define Relay_5_Pin GPIO_PIN_12
#define Relay_5_GPIO_Port GPIOE
#define Relay_6_Pin GPIO_PIN_13
#define Relay_6_GPIO_Port GPIOE
#define Relay_7_Pin GPIO_PIN_14
#define Relay_7_GPIO_Port GPIOE
#define Relay_8_Pin GPIO_PIN_15
#define Relay_8_GPIO_Port GPIOE
#define DS18B20_Pin GPIO_PIN_12
#define DS18B20_GPIO_Port GPIOB
#define DS18B20_Vcc_Pin GPIO_PIN_13
#define DS18B20_Vcc_GPIO_Port GPIOB
#define PWM_Fan1_Pin GPIO_PIN_12
#define PWM_Fan1_GPIO_Port GPIOD
#define PWM_Fan2_Pin GPIO_PIN_13
#define PWM_Fan2_GPIO_Port GPIOD
#define PWM_Fan3_Pin GPIO_PIN_14
#define PWM_Fan3_GPIO_Port GPIOD
#define PWM_Fan4_Pin GPIO_PIN_15
#define PWM_Fan4_GPIO_Port GPIOD
#define D4_Pin GPIO_PIN_8
#define D4_GPIO_Port GPIOC
#define D8_Pin GPIO_PIN_9
#define D8_GPIO_Port GPIOC
#define A1_Pin GPIO_PIN_8
#define A1_GPIO_Port GPIOA
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
