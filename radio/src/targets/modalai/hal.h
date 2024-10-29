/*
 * Copyright (C) EdgeTX
 *
 * Based on code named
 *   opentx - https://github.com/opentx/opentx
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#pragma once

// #warning TODO: Verify/Set these numbers
#define CPU_FREQ            120000000
#define PERI1_FREQUENCY     30000000
#define PERI2_FREQUENCY     60000000
#define TIMER_MULT_APB1     2
#define TIMER_MULT_APB2     2

#define TELEMETRY_EXTI_PRIO             0 // required for soft serial

// Keys
#if defined(PCBFC2)
  // #warning Add other keys
  #define KEYS_GPIO_REG_PAGEUP          GPIOA
  #define KEYS_GPIO_PIN_PAGEUP          LL_GPIO_PIN_11 // PA.11
  #define KEYS_GPIO_REG_PAGEDN          GPIOA
  #define KEYS_GPIO_PIN_PAGEDN          LL_GPIO_PIN_12 // PA.12
  #define KEYS_GPIO_REG_EXIT            GPIOC
  #define KEYS_GPIO_PIN_EXIT            LL_GPIO_PIN_12 // PC.12
  #define KEYS_GPIO_REG_ENTER           GPIOD
  #define KEYS_GPIO_PIN_ENTER           LL_GPIO_PIN_2  // PD.02
  #define KEYS_GPIO_REG_SYS             GPIOC
  #define KEYS_GPIO_PIN_SYS             LL_GPIO_PIN_9  // PC.09
  #define KEYS_GPIO_REG_MDL             GPIOC
  #define KEYS_GPIO_PIN_MDL             LL_GPIO_PIN_8  // PC.08
  #define KEYS_GPIO_REG_TELE            GPIOB
  #define KEYS_GPIO_PIN_TELE            LL_GPIO_PIN_12  // PB.12
#else
  #error Incorrect PCB
#endif

// Rotary Encoder
#if defined(RADIO_MODAL)
  // #warning Check these too
  #define ROTARY_ENCODER_NAVIGATION
  #define ROTARY_ENCODER_GPIO              GPIOH
  #define ROTARY_ENCODER_GPIO_PIN_A        LL_GPIO_PIN_6 // PH.06
  #define ROTARY_ENCODER_GPIO_PIN_B        LL_GPIO_PIN_9 // PH.09
  #define ROTARY_ENCODER_POSITION()        (((ROTARY_ENCODER_GPIO->IDR >> 8) & 0x02) + ((ROTARY_ENCODER_GPIO->IDR >> 6) & 0x01))
  #define ROTARY_ENCODER_EXTI_LINE1        LL_EXTI_LINE_6
  #define ROTARY_ENCODER_EXTI_LINE2        LL_EXTI_LINE_9
  #define ROTARY_ENCODER_EXTI_PORT         LL_SYSCFG_EXTI_PORTH
  #define ROTARY_ENCODER_EXTI_SYS_LINE1    LL_SYSCFG_EXTI_LINE6
  #define ROTARY_ENCODER_EXTI_SYS_LINE2    LL_SYSCFG_EXTI_LINE9
  // ROTARY_ENCODER_EXTI_LINE1 IRQ
  #if !defined(USE_EXTI9_5_IRQ)
    #define USE_EXTI9_5_IRQ
    #define EXTI9_5_IRQ_Priority 5
  #endif
  #define ROTARY_ENCODER_INVERTED
#else
  #error Incorrect Radio
#endif

#if defined(ROTARY_ENCODER_NAVIGATION)
  #define ROTARY_ENCODER_TIMER            TIM5
  #define ROTARY_ENCODER_TIMER_IRQn       TIM5_IRQn
  #define ROTARY_ENCODER_TIMER_IRQHandler TIM5_IRQHandler
#endif

// Trims
#if defined(PCBFC2)
  // #warning Disconnect all of these
  #define TRIMS_GPIO_REG_LHL            GPIOH
  #define TRIMS_GPIO_PIN_LHL            LL_GPIO_PIN_15 // PH.15
  #define TRIMS_GPIO_REG_LHR            GPIOH
  #define TRIMS_GPIO_PIN_LHR            LL_GPIO_PIN_15 // PH.15
  #define TRIMS_GPIO_REG_LVD            GPIOH
  #define TRIMS_GPIO_PIN_LVD            LL_GPIO_PIN_15 // PH.15
  #define TRIMS_GPIO_REG_LVU            GPIOH
  #define TRIMS_GPIO_PIN_LVU            LL_GPIO_PIN_15 // PH.15
  #define TRIMS_GPIO_REG_RVD            GPIOH
  #define TRIMS_GPIO_PIN_RVD            LL_GPIO_PIN_15 // PH.15
  #define TRIMS_GPIO_REG_RHL            GPIOH
  #define TRIMS_GPIO_PIN_RHL            LL_GPIO_PIN_15 // PH.15
  #define TRIMS_GPIO_REG_RVU            GPIOH
  #define TRIMS_GPIO_PIN_RVU            LL_GPIO_PIN_15 // PH.15
  #define TRIMS_GPIO_REG_RHR            GPIOH
  #define TRIMS_GPIO_PIN_RHR            LL_GPIO_PIN_15 // PH.15
#endif

// Switches
#if defined(RADIO_MODAL)
  // #warning Reconnect this
  #define STORAGE_SWITCH_A
  #define HARDWARE_SWITCH_A
  #define SWITCHES_GPIO_REG_A           GPIOG
  #define SWITCHES_GPIO_PIN_A           LL_GPIO_PIN_2  // PG.02
#endif

#if defined(RADIO_MODAL)
  // #warning Check this too
  #define STORAGE_SWITCH_B
  #define HARDWARE_SWITCH_B
  #define SWITCHES_GPIO_REG_B_L         GPIOI
  #define SWITCHES_GPIO_PIN_B_L         LL_GPIO_PIN_0 // PI.00
  #define SWITCHES_GPIO_REG_B_H         GPIOH
  #define SWITCHES_GPIO_PIN_B_H         LL_GPIO_PIN_12  // PH.12
#endif

#if defined(RADIO_MODAL)
  // #warning Define this
  #define STORAGE_SWITCH_C
  #define HARDWARE_SWITCH_C
  #define SWITCHES_GPIO_REG_C_L         GPIOH
  #define SWITCHES_GPIO_PIN_C_L         LL_GPIO_PIN_11  // PH.11
  #define SWITCHES_GPIO_REG_C_H         GPIOH
  #define SWITCHES_GPIO_PIN_C_H         LL_GPIO_PIN_10  // PH.10
#endif

#if defined(RADIO_MODAL)
  // #warning More switches
  #define STORAGE_SWITCH_D
  #define HARDWARE_SWITCH_D
  #define SWITCHES_GPIO_REG_D           GPIOG
  #define SWITCHES_GPIO_PIN_D           LL_GPIO_PIN_3  // PG.3
#endif

#if defined(RADIO_MODAL)
  // #warning And more
  #define STORAGE_SWITCH_E
  #define HARDWARE_SWITCH_E
  #define SWITCHES_GPIO_REG_E           GPIOD
  #define SWITCHES_GPIO_PIN_E           LL_GPIO_PIN_13  // PD.13
#endif

#if defined(RADIO_MODAL)
  // #warning and more
  #define STORAGE_SWITCH_F
  #define HARDWARE_SWITCH_F
  #define SWITCHES_GPIO_REG_F           GPIOD
  #define SWITCHES_GPIO_PIN_F           LL_GPIO_PIN_14 // PD.14
#endif

#if defined(RADIO_MODAL)
  // no SWG
#endif

#if defined(RADIO_MODAL)
  // #warning and this
  #define STORAGE_SWITCH_H
  #define HARDWARE_SWITCH_H
  #define SWITCHES_GPIO_REG_H           GPIOG
  #define SWITCHES_GPIO_PIN_H           LL_GPIO_PIN_4 // PG.4
#endif

#define ADC_SAMPTIME                    LL_ADC_SAMPLINGTIME_8CYCLES_5
#define ADC_CHANNEL_RTC_BAT             LL_ADC_CHANNEL_VBAT

#if defined(PCBFC2)
  // #warning ADC Settings
  #define ADC_MAIN                      ADC1
  #define ADC_DMA                       DMA1
  #define ADC_DMA_CHANNEL               LL_DMAMUX1_REQ_ADC1
  #define ADC_DMA_STREAM                LL_DMA_STREAM_0
  #define ADC_DMA_STREAM_IRQ            DMA1_Stream0_IRQn
  #define ADC_DMA_STREAM_IRQHandler     DMA1_Stream0_IRQHandler
  // #define HARDWARE_POT1 Not enough exposed pins to use these
  // #define HARDWARE_POT2
  #define ADC_GPIO_PIN_STICK_RV         LL_GPIO_PIN_12  // PF.12
  #define ADC_GPIO_PIN_STICK_RH         LL_GPIO_PIN_0  // PB.00
  #define ADC_GPIO_PIN_STICK_LV         LL_GPIO_PIN_0  // PA.00
  #define ADC_GPIO_PIN_STICK_LH         LL_GPIO_PIN_4  // PA.04
  #define ADC_CHANNEL_STICK_RV          LL_ADC_CHANNEL_6  // ADC1_CH6
  #define ADC_CHANNEL_STICK_RH          LL_ADC_CHANNEL_9  // ADC1_CH9
  #define ADC_CHANNEL_STICK_LV          LL_ADC_CHANNEL_16  // ADC1_CH16
  #define ADC_CHANNEL_STICK_LH          LL_ADC_CHANNEL_18  // ADC1_CH18
  // #define ADC_GPIO_PIN_POT1             LL_GPIO_PIN_0  // PB.00
  // #define ADC_GPIO_PIN_POT2             LL_GPIO_PIN_6  // PA.06
  #define ADC_GPIO_PIN_BATT             LL_GPIO_PIN_0  // PC.00 // This pin is disconnected (hopefully that doesn't trigger anything)
  #define ADC_GPIOA_PINS                (ADC_GPIO_PIN_STICK_LH | ADC_GPIO_PIN_STICK_LV)
  #define ADC_GPIOB_PINS                ADC_GPIO_PIN_STICK_RH
  #define ADC_GPIOC_PINS                ADC_GPIO_PIN_BATT
  #define ADC_GPIOF_PINS                ADC_GPIO_PIN_STICK_RV
  // #define ADC_GPIOB_PINS                ADC_GPIO_PIN_POT1
  // #define ADC_CHANNEL_POT1              LL_ADC_CHANNEL_8
  // #define ADC_CHANNEL_POT2              LL_ADC_CHANNEL_6
  #define ADC_CHANNEL_BATT              LL_ADC_CHANNEL_10
  #define ADC_VREF_PREC2                330
  #define ADC_DIRECTION {-1, 1, 1, -1, -1, 1, 1, 1}
#endif

// PWR and LED driver

#if defined(RADIO_MODAL)
  // #warning unsure what these do
  #define PWR_SWITCH_GPIO               GPIO_PIN(GPIOC, 6)  // PC.06
  #define PWR_ON_GPIO                   GPIO_PIN(GPIOC, 7)  // PC.07
#endif

#if defined(RADIO_MODAL)
  // #warning remove these later
  #define STATUS_LEDS
  #define GPIO_LED_GPIO_ON              gpio_clear
  #define GPIO_LED_GPIO_OFF             gpio_set
  #define LED_GREEN_GPIO                GPIO_PIN(GPIOE, 4)  // PE.04
  #define LED_RED_GPIO                  GPIO_PIN(GPIOE, 3) // PE.03
  #define LED_BLUE_GPIO                 GPIO_PIN(GPIOE, 5)  // PA.05
#endif


// Internal Module
#if defined(RADIO_MODAL)
  // #warning definitely check on these
  #define INTMODULE_BOOTCMD_GPIO           GPIO_PIN(GPIOB, 8) // PB.08
  #define INTMODULE_PWR_GPIO               GPIO_PIN(GPIOB, 9) // PB.09
  #define INTMODULE_BOOTCMD_DEFAULT     0 // RESET
  #define INTMODULE_TX_GPIO                GPIO_PIN(GPIOB, 6) // PB.06
  #define INTMODULE_RX_GPIO                GPIO_PIN(GPIOB, 7) // PB.07
  #define INTMODULE_USART                  USART1
  #define INTMODULE_USART_IRQHandler       USART1_IRQHandler
  #define INTMODULE_USART_IRQn             USART1_IRQn
  // #define INTMODULE_DMA                    DMA2
  // #define INTMODULE_DMA_STREAM             LL_DMA_STREAM_7
  // #define INTMODULE_DMA_STREAM_IRQ         DMA2_Stream7_IRQn
  // #define INTMODULE_DMA_STREAM_IRQHandler  DMA2_Stream7_IRQHandler
  // #define INTMODULE_DMA_CHANNEL            LL_DMA_CHANNEL_4
  // #define INTMODULE_RX_DMA                 DMA2
  // #define INTMODULE_RX_DMA_STREAM          LL_DMA_STREAM_2
  // #define INTMODULE_RX_DMA_CHANNEL         LL_DMA_CHANNEL_4
#endif

// External Module
#if defined(RADIO_MODAL)
  // #warning Remove all EXTMODULE references
#endif


#if defined(MODAL_RADIO)
  // #warning Make sure this removes TELEMETRY completely (the pins are used elsewhere)
  #undef TELEMETRY_USART
#endif

// Software IRQ (Prio 5 -> FreeRTOS compatible)
//#define TELEMETRY_RX_FRAME_EXTI_LINE    LL_EXTI_LINE_4
//#define USE_EXTI4_IRQ
//#define EXTI4_IRQ_Priority 5

// USB Charger
#if defined(USB_CHARGER)
  #error should be off
  #define USB_CHARGER_GPIO              GPIO_PIN(GPIOB, 5)
#endif

#define SPORT_MAX_BAUDRATE            400000

// USB
#define USB_GPIO_VBUS                   GPIO_PIN(GPIOA, 9)  // PA.09
#define USB_GPIO_DM                     GPIO_PIN(GPIOA, 11) // PA.11
#define USB_GPIO_DP                     GPIO_PIN(GPIOA, 12) // PA.12
#define USB_GPIO_AF                     GPIO_AF10

// BackLight
#if defined(PCBFC2)
  // #warning Backlight settings
  #define BACKLIGHT_TIMER_FREQ          (PERI1_FREQUENCY * TIMER_MULT_APB1)
  #define BACKLIGHT_TIMER               TIM3
  #define BACKLIGHT_GPIO                GPIO_PIN(GPIOA, 6) // PA.06
  #define BACKLIGHT_GPIO_AF             GPIO_AF2
  #define BACKLIGHT_CCMR1               TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 // Channel1, PWM
  #define BACKLIGHT_CCER                TIM_CCER_CC1E
  #define BACKLIGHT_COUNTER_REGISTER    BACKLIGHT_TIMER->CCR1
#endif

// LCD driver
#if defined(RADIO_MODAL)
  // #warning LCD settings
  #define LCD_MOSI_GPIO                 GPIO_PIN(GPIOG, 14) // PG.14
  #define LCD_CLK_GPIO                  GPIO_PIN(GPIOB, 3) // PB.03
  #define LCD_A0_GPIO                   GPIO_PIN(GPIOF, 0) // PF.00
  #define LCD_NCS_GPIO                  GPIO_PIN(GPIOI, 10) // PI.10
  #define LCD_RST_GPIO                  GPIO_PIN(GPIOF, 1) // PF.01
  #define LCD_DMA                       BDMA
  #define LCD_DMA_Stream                BDMA_Channel0
  #define LCD_DMA_Stream_IRQn           BDMA_Channel0_IRQn
  #define LCD_DMA_Stream_IRQHandler     BDMA_Channel0_IRQHandler
  #define LCD_DMAMUX                    DMAMUX2_Channel0
  #define LCD_DMA_FLAGS                 ( BDMA_IFCR_CGIF0 | BDMA_IFCR_CTCIF0 | BDMA_IFCR_CHTIF0 | BDMA_IFCR_CTEIF0)
  #define LCD_DMA_FLAG_INT              BDMA_IFCR_CGIF0
  #define LCD_SPI                       SPI6
  #define LCD_GPIO_AF                   GPIO_AF5 // For SPI6
  #define LCD_VERTICAL_INVERT
  #define LCD_SPI_PRESCALER             0x20000000
#endif

// I2C Bus 1: EEPROM and CAT5137 digital pot for volume control
// #define I2C_B1                          I2C1
// #define I2C_B1_GPIO_AF                  LL_GPIO_AF_4

// #warning remove eeprom
// #define I2C_B1_SCL_GPIO               GPIO_PIN(GPIOB, 8)  // PB.08
// #define I2C_B1_SDA_GPIO               GPIO_PIN(GPIOB, 9)  // PB.09

// EEPROM
// #warning remove eeprom
// #define EEPROM_WP_GPIO                GPIOD
// #define EEPROM_WP_GPIO_PIN            LL_GPIO_PIN_10  // PD.10

// I2C Volume control
#if !defined(SOFTWARE_VOLUME)
  #define VOLUME_I2C_ADDRESS            0x2E
  #define VOLUME_I2C_BUS                I2C_Bus_1
#endif

// #define I2C_B1_CLK_RATE                 400000

// EEPROM
#if defined(RADIO_MODAL)
// #warning remove this
//#define EEPROM_I2C_ADDRESS              0x51
//#define EEPROM_I2C_BUS                  I2C_Bus_1
//#define EEPROM_PAGESIZE                 64
//#define EEPROM_SIZE                     (32*1024)
#endif

// Second I2C Bus: IMU
// TODO: We do have a I2C IMU on the flightcore?? If time allows might as well integrate it...
#if defined(PCBXLITES)
  #define I2C_B2                        I2C3
  #define I2C_B2_SCL_GPIO               GPIO_PIN(GPIOA, 8) // PA.08
  #define I2C_B2_SDA_GPIO               GPIO_PIN(GPIOC, 9) // PC.09
  #define I2C_B2_GPIO_AF                LL_GPIO_AF_4
  #define I2C_B2_CLK_RATE               400000

  #define IMU_I2C_BUS                   I2C_Bus_2
  #define IMU_I2C_ADDRESS               0x6B
#endif

#if defined(RADIO_MODAL)
//SD - SDIO
// #warning don't use DMA, use IDMA on peripheral
// #define SD_SDIO_DMA                     DMA2
// #define SD_SDIO_DMA_STREAM              DMA2_Stream3
// #define SD_SDIO_DMA_CHANNEL             LL_DMA_CHANNEL_4
// #define SD_SDIO_DMA_IRQn                DMA2_Stream3_IRQn
// #define SD_SDIO_DMA_IRQHANDLER          DMA2_Stream3_IRQHandler
// #define SD_SDIO_DMAMUX                  DMAMUX1_Channel2
#define SD_SDIO_CLK_DIV(fq)             ((48000000 / (fq)) - 2)
#define SD_SDIO_INIT_CLK_DIV            SD_SDIO_CLK_DIV(400000)
#define SD_SDIO_TRANSFER_CLK_DIV        SD_SDIO_CLK_DIV(24000000)
#define STORAGE_USE_SDIO

// Define the pins just to be sure
#define SD_SDIO_PIN_D0 GPIO_PIN(GPIOB, 14) // PB.14
#define SD_SDIO_AF_D0 GPIO_AF11
#define SD_SDIO_PIN_D1 GPIO_PIN(GPIOB, 15) // PB.15
#define SD_SDIO_AF_D1 GPIO_AF11
#define SD_SDIO_PIN_D2 GPIO_PIN(GPIOG, 11) // PG.11
#define SD_SDIO_AF_D2 GPIO_AF11
#define SD_SDIO_PIN_D3 GPIO_PIN(GPIOB, 4) // PB.04
#define SD_SDIO_AF_D3 GPIO_AF11
#define SD_SDIO_PIN_CMD GPIO_PIN(GPIOD, 7) // PD.07
#define SD_SDIO_AF_CMD GPIO_AF11
#define SD_SDIO_PIN_CLK GPIO_PIN(GPIOD, 6) // PD.06
#define SD_SDIO_AF_CLK GPIO_AF11
// Use SDMMC2
#define SD_SDIO SDMMC2
#define SD_SDIO_2

#endif // Radio_Modal SD

// Audio
// #warning Disable audio and haptics
#define HAPTIC_GPIO                   GPIO_PIN(GPIOC, 12) // PC.12

// To avoid change in modelsize, todo: remove me
#define STORAGE_BLUETOOTH

// Millisecond timer
#define MS_TIMER                        TIM14
#define MS_TIMER_IRQn                   TIM8_TRG_COM_TIM14_IRQn
#define MS_TIMER_IRQHandler             TIM8_TRG_COM_TIM14_IRQHandler

// Mixer scheduler timer
#define MIXER_SCHEDULER_TIMER                TIM12
#define MIXER_SCHEDULER_TIMER_FREQ           (PERI1_FREQUENCY * TIMER_MULT_APB1)
#define MIXER_SCHEDULER_TIMER_IRQn           TIM8_BRK_TIM12_IRQn
#define MIXER_SCHEDULER_TIMER_IRQHandler     TIM8_BRK_TIM12_IRQHandler
