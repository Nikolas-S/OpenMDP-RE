/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_e_Pin GPIO_PIN_2
#define BTN_e_GPIO_Port GPIOE
#define BTN_e_EXTI_IRQn EXTI2_IRQn
#define BTN_d_Pin GPIO_PIN_3
#define BTN_d_GPIO_Port GPIOE
#define BTN_d_EXTI_IRQn EXTI3_IRQn
#define BTN_c_Pin GPIO_PIN_4
#define BTN_c_GPIO_Port GPIOE
#define BTN_c_EXTI_IRQn EXTI4_IRQn
#define BTN_b_Pin GPIO_PIN_5
#define BTN_b_GPIO_Port GPIOE
#define BTN_b_EXTI_IRQn EXTI9_5_IRQn
#define BTN_a_Pin GPIO_PIN_6
#define BTN_a_GPIO_Port GPIOE
#define BTN_a_EXTI_IRQn EXTI9_5_IRQn
#define ENC_menu_b_Pin GPIO_PIN_13
#define ENC_menu_b_GPIO_Port GPIOC
#define ENC_menu_b_EXTI_IRQn EXTI15_10_IRQn
#define NC_PC14_Pin GPIO_PIN_14
#define NC_PC14_GPIO_Port GPIOC
#define ENC_menu_a_Pin GPIO_PIN_15
#define ENC_menu_a_GPIO_Port GPIOC
#define ENC_menu_a_EXTI_IRQn EXTI15_10_IRQn
#define ENC_value_a_Pin GPIO_PIN_0
#define ENC_value_a_GPIO_Port GPIOC
#define ENC_value_a_EXTI_IRQn EXTI0_IRQn
#define ENC_value_b_Pin GPIO_PIN_1
#define ENC_value_b_GPIO_Port GPIOC
#define ENC_value_b_EXTI_IRQn EXTI1_IRQn
#define NC_PC2_Pin GPIO_PIN_2
#define NC_PC2_GPIO_Port GPIOC
#define NC_PC3_Pin GPIO_PIN_3
#define NC_PC3_GPIO_Port GPIOC
#define NC_PA0_Pin GPIO_PIN_0
#define NC_PA0_GPIO_Port GPIOA
#define NC_PA1_Pin GPIO_PIN_1
#define NC_PA1_GPIO_Port GPIOA
#define NC_PA2_Pin GPIO_PIN_2
#define NC_PA2_GPIO_Port GPIOA
#define NC_PA3_Pin GPIO_PIN_3
#define NC_PA3_GPIO_Port GPIOA
#define NC_PA4_Pin GPIO_PIN_4
#define NC_PA4_GPIO_Port GPIOA
#define NC_PA5_Pin GPIO_PIN_5
#define NC_PA5_GPIO_Port GPIOA
#define NC_PA6_Pin GPIO_PIN_6
#define NC_PA6_GPIO_Port GPIOA
#define NC_PA7_Pin GPIO_PIN_7
#define NC_PA7_GPIO_Port GPIOA
#define NC_PC4_Pin GPIO_PIN_4
#define NC_PC4_GPIO_Port GPIOC
#define NC_PC5_Pin GPIO_PIN_5
#define NC_PC5_GPIO_Port GPIOC
#define NC_PB0_Pin GPIO_PIN_0
#define NC_PB0_GPIO_Port GPIOB
#define LCD_reset_AL_Pin GPIO_PIN_1
#define LCD_reset_AL_GPIO_Port GPIOB
#define GND_PB2_Pin GPIO_PIN_2
#define GND_PB2_GPIO_Port GPIOB
#define LCD_d4_Pin GPIO_PIN_7
#define LCD_d4_GPIO_Port GPIOE
#define LCD_d5_Pin GPIO_PIN_8
#define LCD_d5_GPIO_Port GPIOE
#define LCD_d6_Pin GPIO_PIN_9
#define LCD_d6_GPIO_Port GPIOE
#define LCD_d7_Pin GPIO_PIN_10
#define LCD_d7_GPIO_Port GPIOE
#define LCD_d8_Pin GPIO_PIN_11
#define LCD_d8_GPIO_Port GPIOE
#define LCD_d9_Pin GPIO_PIN_12
#define LCD_d9_GPIO_Port GPIOE
#define LCD_d10_Pin GPIO_PIN_13
#define LCD_d10_GPIO_Port GPIOE
#define LCD_d11_Pin GPIO_PIN_14
#define LCD_d11_GPIO_Port GPIOE
#define LCD_d12_Pin GPIO_PIN_15
#define LCD_d12_GPIO_Port GPIOE
#define NC_PB10_Pin GPIO_PIN_10
#define NC_PB10_GPIO_Port GPIOB
#define LCD_backlight_sink_AH_Pin GPIO_PIN_11
#define LCD_backlight_sink_AH_GPIO_Port GPIOB
#define SPI2_FLASH_cs_Pin GPIO_PIN_12
#define SPI2_FLASH_cs_GPIO_Port GPIOB
#define SPI2_FLASH_clk_Pin GPIO_PIN_13
#define SPI2_FLASH_clk_GPIO_Port GPIOB
#define SPI2_FLASH_miso_Pin GPIO_PIN_14
#define SPI2_FLASH_miso_GPIO_Port GPIOB
#define SPI2_FLASH_mosi_Pin GPIO_PIN_15
#define SPI2_FLASH_mosi_GPIO_Port GPIOB
#define LCD_d13_Pin GPIO_PIN_8
#define LCD_d13_GPIO_Port GPIOD
#define LCD_d14_Pin GPIO_PIN_9
#define LCD_d14_GPIO_Port GPIOD
#define LCD_d15_Pin GPIO_PIN_10
#define LCD_d15_GPIO_Port GPIOD
#define PWR_periph_en_AL_Pin GPIO_PIN_11
#define PWR_periph_en_AL_GPIO_Port GPIOD
#define NC_PD12_Pin GPIO_PIN_12
#define NC_PD12_GPIO_Port GPIOD
#define LCD_a18_rs_Pin GPIO_PIN_13
#define LCD_a18_rs_GPIO_Port GPIOD
#define LCD_d0_Pin GPIO_PIN_14
#define LCD_d0_GPIO_Port GPIOD
#define LCD_d1_Pin GPIO_PIN_15
#define LCD_d1_GPIO_Port GPIOD
#define NC_PC6_Pin GPIO_PIN_6
#define NC_PC6_GPIO_Port GPIOC
#define NC_PC7_Pin GPIO_PIN_7
#define NC_PC7_GPIO_Port GPIOC
#define NC_PC8_Pin GPIO_PIN_8
#define NC_PC8_GPIO_Port GPIOC
#define NC_PC9_Pin GPIO_PIN_9
#define NC_PC9_GPIO_Port GPIOC
#define NC_PA8_Pin GPIO_PIN_8
#define NC_PA8_GPIO_Port GPIOA
#define NC_PA9_Pin GPIO_PIN_9
#define NC_PA9_GPIO_Port GPIOA
#define USB_presence_Pin GPIO_PIN_10
#define USB_presence_GPIO_Port GPIOA
#define USB_dm_Pin GPIO_PIN_11
#define USB_dm_GPIO_Port GPIOA
#define USB_dp_Pin GPIO_PIN_12
#define USB_dp_GPIO_Port GPIOA
#define SWD_dio_Pin GPIO_PIN_13
#define SWD_dio_GPIO_Port GPIOA
#define SWD_clk_Pin GPIO_PIN_14
#define SWD_clk_GPIO_Port GPIOA
#define SPI1_RF_cs_Pin GPIO_PIN_15
#define SPI1_RF_cs_GPIO_Port GPIOA
#define TP_PC10_Pin GPIO_PIN_10
#define TP_PC10_GPIO_Port GPIOC
#define TP_PC11_Pin GPIO_PIN_11
#define TP_PC11_GPIO_Port GPIOC
#define NC_PC12_Pin GPIO_PIN_12
#define NC_PC12_GPIO_Port GPIOC
#define LCD_d2_Pin GPIO_PIN_0
#define LCD_d2_GPIO_Port GPIOD
#define LCD_d3_Pin GPIO_PIN_1
#define LCD_d3_GPIO_Port GPIOD
#define RF_ce_Pin GPIO_PIN_2
#define RF_ce_GPIO_Port GPIOD
#define NC_PD3_Pin GPIO_PIN_3
#define NC_PD3_GPIO_Port GPIOD
#define LCD_noe_rd_AL_Pin GPIO_PIN_4
#define LCD_noe_rd_AL_GPIO_Port GPIOD
#define LCD_nwe_wr_AL_Pin GPIO_PIN_5
#define LCD_nwe_wr_AL_GPIO_Port GPIOD
#define RF_irq_Pin GPIO_PIN_6
#define RF_irq_GPIO_Port GPIOD
#define LCD_ne1_cs_AL_Pin GPIO_PIN_7
#define LCD_ne1_cs_AL_GPIO_Port GPIOD
#define SPI1_RF_clk_Pin GPIO_PIN_3
#define SPI1_RF_clk_GPIO_Port GPIOB
#define SPI1_RF_miso_Pin GPIO_PIN_4
#define SPI1_RF_miso_GPIO_Port GPIOB
#define SPI1_RF_mosi_Pin GPIO_PIN_5
#define SPI1_RF_mosi_GPIO_Port GPIOB
#define LED_blue_AL_Pin GPIO_PIN_6
#define LED_blue_AL_GPIO_Port GPIOB
#define NC_PB7_Pin GPIO_PIN_7
#define NC_PB7_GPIO_Port GPIOB
#define LED_red_AL_Pin GPIO_PIN_8
#define LED_red_AL_GPIO_Port GPIOB
#define BUZZER_AH_Pin GPIO_PIN_9
#define BUZZER_AH_GPIO_Port GPIOB
#define NC_PE0_Pin GPIO_PIN_0
#define NC_PE0_GPIO_Port GPIOE
#define NC_PE1_Pin GPIO_PIN_1
#define NC_PE1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
