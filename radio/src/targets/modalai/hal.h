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

#define CPU_FREQ                400000000

#define PERI1_FREQUENCY         100000000
#define PERI2_FREQUENCY         100000000
#define TIMER_MULT_APB1         2
#define TIMER_MULT_APB2         2

#define TELEMETRY_EXTI_PRIO             0 // required for soft serial

/* Timers Allocation:
 * TIM7 = Rotary Encoder
 *
 * TIM5 = Backlight
 *
 * TIM12 = Mixer scheduler
 * TIM14 = 5 ms counter (unused)
 */

/* DMA Allocation:
   DMA/Stream/Channel
   1/7/0? LCD
   1/0/0 ADC1
   2/3/4 SDIO
*/

// Zorro Blue official hardware config

// Keys

#define KEYS_GPIO_REG_PAGEUP          GPIOF
#define KEYS_GPIO_PIN_PAGEUP          LL_GPIO_PIN_13 // PF.13
#define KEYS_GPIO_REG_PAGEDN          GPIOG
#define KEYS_GPIO_PIN_PAGEDN          LL_GPIO_PIN_0 // PG.00
#define KEYS_GPIO_REG_EXIT            GPIOG
#define KEYS_GPIO_PIN_EXIT            LL_GPIO_PIN_1 // PG.01
#define KEYS_GPIO_REG_ENTER           GPIOD
#define KEYS_GPIO_PIN_ENTER           LL_GPIO_PIN_14  // PD.14
#define KEYS_GPIO_REG_SYS             GPIOB
#define KEYS_GPIO_PIN_SYS             LL_GPIO_PIN_2  // PB.02
#define KEYS_GPIO_REG_MDL             GPIOH
#define KEYS_GPIO_PIN_MDL             LL_GPIO_PIN_6  // PH.06
#define KEYS_GPIO_REG_TELE            GPIOH
#define KEYS_GPIO_PIN_TELE            LL_GPIO_PIN_9  // PH.09

// Rotary Encoder

#define ROTARY_ENCODER_NAVIGATION
#define ROTARY_ENCODER_GPIO              GPIOD
#define ROTARY_ENCODER_GPIO_PIN_A        LL_GPIO_PIN_11 // PD.11
#define ROTARY_ENCODER_GPIO_PIN_B        LL_GPIO_PIN_10 // PD.10
#define ROTARY_ENCODER_POSITION()        (((ROTARY_ENCODER_GPIO->IDR >> 10) & 0x02) + ((ROTARY_ENCODER_GPIO->IDR >> 10) & 0x01))
#define ROTARY_ENCODER_EXTI_LINE1        LL_EXTI_LINE_11
#define ROTARY_ENCODER_EXTI_LINE2        LL_EXTI_LINE_10
#define ROTARY_ENCODER_EXTI_PORT         LL_SYSCFG_EXTI_PORTD
#define ROTARY_ENCODER_EXTI_SYS_LINE1    LL_SYSCFG_EXTI_LINE11
#define ROTARY_ENCODER_EXTI_SYS_LINE2    LL_SYSCFG_EXTI_LINE10
// ROTARY_ENCODER_EXTI_LINE1 IRQ
#if !defined(USE_EXTI15_10_IRQ)
  #define USE_EXTI15_10_IRQ
  #define EXTI15_10_IRQ_Priority 5
#endif
#define ROTARY_ENCODER_INVERTED

#define ROTARY_ENCODER_TIMER            TIM7
#define ROTARY_ENCODER_TIMER_IRQn       TIM7_IRQn
#define ROTARY_ENCODER_TIMER_IRQHandler TIM7_IRQHandler


// Switches

#define STORAGE_SWITCH_A
#define HARDWARE_SWITCH_A
#define SWITCHES_GPIO_REG_A_L         GPIOA
#define SWITCHES_GPIO_PIN_A_L         LL_GPIO_PIN_1  // PA.01
#define SWITCHES_GPIO_REG_A_H         GPIOC
#define SWITCHES_GPIO_PIN_A_H         LL_GPIO_PIN_4  // PC.04

#define STORAGE_SWITCH_B
#define HARDWARE_SWITCH_B
#define SWITCHES_GPIO_REG_B_L         GPIOH
#define SWITCHES_GPIO_PIN_B_L         LL_GPIO_PIN_11  // PH.11
#define SWITCHES_GPIO_REG_B_H         GPIOH
#define SWITCHES_GPIO_PIN_B_H         LL_GPIO_PIN_12  // PH.12

#define STORAGE_SWITCH_C
#define HARDWARE_SWITCH_C
#define SWITCHES_GPIO_REG_C_L         GPIOD
#define SWITCHES_GPIO_PIN_C_L         LL_GPIO_PIN_12  // PD.12
#define SWITCHES_GPIO_REG_C_H         GPIOD
#define SWITCHES_GPIO_PIN_C_H         LL_GPIO_PIN_13  // PD.13

#define STORAGE_SWITCH_D
#define HARDWARE_SWITCH_D
#define SWITCHES_GPIO_REG_D_L         GPIOB
#define SWITCHES_GPIO_PIN_D_L         LL_GPIO_PIN_1  // PB.01
#define SWITCHES_GPIO_REG_D_H         GPIOE
#define SWITCHES_GPIO_PIN_D_H         LL_GPIO_PIN_10  // PE.10

#define STORAGE_SWITCH_E
#define HARDWARE_SWITCH_E
#define SWITCHES_GPIO_REG_E           GPIOF
#define SWITCHES_GPIO_PIN_E           LL_GPIO_PIN_10  // PF.10

#define STORAGE_SWITCH_F
#define HARDWARE_SWITCH_F
#define SWITCHES_GPIO_REG_F           GPIOE
#define SWITCHES_GPIO_PIN_F           LL_GPIO_PIN_12  // PE.12

// Sticks and pots

#define ADC_SAMPTIME                  LL_ADC_SAMPLINGTIME_8CYCLES_5
#define ADC_CHANNEL_RTC_BAT           LL_ADC_CHANNEL_VBAT
#define ADC_VREF_PREC2                330

#define ADC_MAIN                      ADC1
#define ADC_DMA                       DMA1
#define ADC_DMA_CHANNEL               LL_DMAMUX1_REQ_ADC1
#define ADC_DMA_STREAM                LL_DMA_STREAM_0
#define ADC_DMA_STREAM_IRQ            DMA1_Stream0_IRQn
#define ADC_DMA_STREAM_IRQHandler     DMA1_Stream0_IRQHandler
#define HARDWARE_POT1
#define HARDWARE_POT2
#define ADC_GPIO_PIN_STICK_RV         LL_GPIO_PIN_11  // PF.11
#define ADC_GPIO_PIN_STICK_RH         LL_GPIO_PIN_12  // PF.12
#define ADC_GPIO_PIN_STICK_LV         LL_GPIO_PIN_1  // PC.01
#define ADC_GPIO_PIN_STICK_LH         LL_GPIO_PIN_0  // PC.00
#define ADC_CHANNEL_STICK_RV          LL_ADC_CHANNEL_2  // ADC1_CH2
#define ADC_CHANNEL_STICK_RH          LL_ADC_CHANNEL_6  // ADC1_CH6
#define ADC_CHANNEL_STICK_LV          LL_ADC_CHANNEL_11  // ADC1_CH11
#define ADC_CHANNEL_STICK_LH          LL_ADC_CHANNEL_10  // ADC1_CH10
#define ADC_CHANNEL_POT2              LL_ADC_CHANNEL_9 // ADC1_CH9
#define ADC_CHANNEL_POT1              LL_ADC_CHANNEL_16 // ADC1_CH16
#define ADC_GPIO_PIN_POT2             LL_GPIO_PIN_0  // PB.00
#define ADC_GPIO_PIN_POT1             LL_GPIO_PIN_0  // PA.00
// !#define ADC_GPIO_PIN_BATT             LL_GPIO_PIN_0  // PC.00 // This pin is disconnected (hopefully that doesn't trigger anything)
#define ADC_GPIOA_PINS                (ADC_GPIO_PIN_POT1)
#define ADC_GPIOB_PINS                (ADC_GPIO_PIN_POT2)
#define ADC_GPIOC_PINS                (ADC_GPIO_PIN_STICK_LH | ADC_GPIO_PIN_STICK_LV)
#define ADC_GPIOF_PINS                (ADC_GPIO_PIN_STICK_RH | ADC_GPIO_PIN_STICK_RV)
// !#define ADC_CHANNEL_BATT              LL_ADC_CHANNEL_10
#define ADC_DIRECTION {-1, 1, 1, -1, 1, -1, 1, 1}

// PWR and LED driver

// #warning unsure what these do, TODO: Remove these buttons completely
// don't define PWR_SWITCH_GPIO to not check on boot
// #define PWR_SWITCH_GPIO               GPIO_PIN(GPIOA, 5)  // PA.06
// don't need to set PWR_ON_GPIO since there's no hardware latch connected like on most button controllers
// #define PWR_ON_GPIO                   GPIO_PIN(GPIOA, 6)  // PA.07

#define USE_LEDS
#ifdef USE_LEDS
#define STATUS_LEDS
#define GPIO_LED_GPIO_ON              gpio_clear
#define GPIO_LED_GPIO_OFF             gpio_set
#define LED_GREEN_GPIO                GPIO_PIN(GPIOI, 7)  // PI.07
#define LED_RED_GPIO                  GPIO_PIN(GPIOI, 6) // PI.06
#define LED_BLUE_GPIO                 GPIO_PIN(GPIOI, 5)  // PI.05
#endif // USE_LEDS

// Internal Module
#define INTMODULE_BOOTCMD_GPIO           GPIO_PIN(GPIOE, 13) // PE.13 (Disconnected)
#define INTMODULE_PWR_GPIO               GPIO_PIN(GPIOG, 8) // PG.08 (Disconnected)
#define INTMODULE_BOOTCMD_DEFAULT     0 // RESET
// Alternate pin assignment depending on the side of M0207 we use
//#define INTMODULE_TX_GPIO                GPIO_PIN(GPIOB, 13) // PB.13
//#define INTMODULE_RX_GPIO                GPIO_PIN(GPIOB, 12) // PB.12
//#define INTMODULE_USART                  UART5
//#define INTMODULE_USART_IRQHandler       UART5_IRQHandler
//#define INTMODULE_USART_IRQn             UART5_IRQn
#define INTMODULE_TX_GPIO                GPIO_PIN(GPIOE, 1) // PE.01
#define INTMODULE_RX_GPIO                GPIO_PIN(GPIOE, 0) // PE.00
#define INTMODULE_USART                  UART8
#define INTMODULE_USART_IRQHandler       UART8_IRQHandler
#define INTMODULE_USART_IRQn             UART8_IRQn
// #define INTMODULE_DMA                    DMA2
// #define INTMODULE_DMA_STREAM             LL_DMA_STREAM_7
// #define INTMODULE_DMA_STREAM_IRQ         DMA2_Stream7_IRQn
// #define INTMODULE_DMA_STREAM_IRQHandler  DMA2_Stream7_IRQHandler
// #define INTMODULE_DMA_CHANNEL            LL_DMA_CHANNEL_4
// #define INTMODULE_RX_DMA                 DMA2
// #define INTMODULE_RX_DMA_STREAM          LL_DMA_STREAM_2
// #define INTMODULE_RX_DMA_CHANNEL         LL_DMA_CHANNEL_4

// TODO: Add extmodule when pins are decided on

// Software IRQ (Prio 5 -> FreeRTOS compatible)
//#define TELEMETRY_RX_FRAME_EXTI_LINE    LL_EXTI_LINE_4
//#define USE_EXTI4_IRQ
//#define EXTI4_IRQ_Priority 5

// USB Charger
#if defined(USB_CHARGER)
  #error should be off
  #define USB_CHARGER_GPIO              GPIO_PIN(GPIOB, 5)
#endif

// TODO: increase
#define SPORT_MAX_BAUDRATE            400000

// USB
#define USB_GPIO_VBUS                   GPIO_PIN(GPIOA, 9)  // PA.09
#define USB_GPIO_DM                     GPIO_PIN(GPIOA, 11) // PA.11
#define USB_GPIO_DP                     GPIO_PIN(GPIOA, 12) // PA.12
#define USB_GPIO_AF                     GPIO_AF10

// BackLight
#define BACKLIGHT_TIMER_FREQ          (PERI1_FREQUENCY * TIMER_MULT_APB1)
#define BACKLIGHT_TIMER               TIM5
#define BACKLIGHT_GPIO                GPIO_PIN(GPIOH, 10) // PH.10
#define BACKLIGHT_GPIO_AF             GPIO_AF2 // TIM5 CH1
#define BACKLIGHT_CCMR1               TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 // Channel1, PWM
#define BACKLIGHT_CCER                TIM_CCER_CC1E
#define BACKLIGHT_COUNTER_REGISTER    BACKLIGHT_TIMER->CCR1

// LCD driver
#define LCD_MOSI_GPIO                 GPIO_PIN(GPIOE, 6) // PE.06
#define LCD_CLK_GPIO                  GPIO_PIN(GPIOE, 2) // PE.02
#define LCD_A0_GPIO                   GPIO_PIN(GPIOE, 4) // PE.04
#define LCD_NCS_GPIO                  GPIO_PIN(GPIOE, 3) // PE.03
#define LCD_RST_GPIO                  GPIO_PIN(GPIOE, 5) // PE.05
#define LCD_DMA                       DMA1
#define LCD_DMA_Stream                DMA1_Stream7
#define LCD_DMA_Stream_Num            LL_DMA_STREAM_7
#define LCD_DMA_Stream_IRQn           DMA1_Stream7_IRQn
#define LCD_DMA_Stream_IRQHandler     DMA1_Stream7_IRQHandler
#define LCD_DMA_FLAGS                 (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7)
#define LCD_DMA_FLAG_INT              DMA_HIFCR_CTCIF7
#define LCD_SPI                       SPI4
#define LCD_GPIO_AF                   GPIO_AF5 // For SPI4
#define LCD_VERTICAL_INVERT
#define LCD_SPI_PRESCALER             0x10000000


// TODO: M0207 has a SPI EEPROM
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

// #define I2C_B1_CLK_RATE                 400000


// FC2 internal hardware

//SD - SDIO
// #define SD_SDIO_DMA                     DMA2
// #define SD_SDIO_DMA_STREAM              DMA2_Stream3
// #define SD_SDIO_DMA_CHANNEL             LL_DMA_CHANNEL_4
// #define SD_SDIO_DMA_IRQn                DMA2_Stream3_IRQn
// #define SD_SDIO_DMA_IRQHANDLER          DMA2_Stream3_IRQHandler
// #define SD_SDIO_DMAMUX                  DMAMUX1_Channel2
// #define SD_SDIO_CLK_DIV(fq)             ((48000000 / (fq)) - 2)
// #define SD_SDIO_INIT_CLK_DIV            SD_SDIO_CLK_DIV(400000)
// #define SD_SDIO_TRANSFER_CLK_DIV        SD_SDIO_CLK_DIV(24000000)
#define STORAGE_USE_SDIO
#define SD_SDIO_TRANSFER_CLK_DIV        SDMMC_NSPEED_CLK_DIV

// Define the pins just to be sure
#define SD_SDIO_PIN_D0 GPIO_PIN(GPIOC, 8) // PC.08
#define SD_SDIO_AF_D0 GPIO_AF12
#define SD_SDIO_PIN_D1 GPIO_PIN(GPIOC, 9) // PC.09
#define SD_SDIO_AF_D1 GPIO_AF12
#define SD_SDIO_PIN_D2 GPIO_PIN(GPIOC, 10) // PC.10
#define SD_SDIO_AF_D2 GPIO_AF12
#define SD_SDIO_PIN_D3 GPIO_PIN(GPIOC, 11) // PC.11
#define SD_SDIO_AF_D3 GPIO_AF12
#define SD_SDIO_PIN_CMD GPIO_PIN(GPIOD, 2) // PD.02
#define SD_SDIO_AF_CMD GPIO_AF12
#define SD_SDIO_PIN_CLK GPIO_PIN(GPIOC, 12) // PC.12
#define SD_SDIO_AF_CLK GPIO_AF12
// Use SDMMC1
#define SD_SDIO SDMMC1

// Power monitoring


#define VOXLPM_INA231_ADDR_VBATT		0x44 << 1
#define VOXLPM_INA231_ADDR_P5_12VDC		0x45 << 1

/* INA231 Registers addresses */
#define INA231_REG_CONFIG			0x00
#define INA231_REG_SHUNTVOLTAGE			0x01
#define INA231_REG_BUSVOLTAGE			0x02
#define INA231_REG_POWER			0x03
#define INA231_REG_CURRENT			0x04
#define INA231_REG_CALIBRATION			0x05
#define INA231_REG_MASKENABLE			0x06
#define INA231_REG_ALERTLIMIT			0x07

/* [0:2] Mode - Shunt and bus, 111, continuous (INA231A default) */
#define INA231_CONFIG_MODE			(0x07 << 0)
/* [5:3] Shunt Voltage Conversion Time, 100, 1.1ms (INA231A default) */
#define INA231_CONFIG_SHUNT_CT			(0x04 << 3)
/* [8:6] Shunt Voltage Conversion Time, 100, 1.1ms (INA231A default) */
#define INA231_CONFIG_BUS_CT			(0x04 << 6)
/* [11:9] Averaging Mode, 010, 16 */
#define INA231_CONFIG_AVG			(0x02 << 9)
/* [1] Reset bit */
#define INA231_RST_BIT				(0x01 << 15)
/* Configuration register settings */
#define INA231_CONFIG				(INA231_CONFIG_MODE+INA231_CONFIG_SHUNT_CT+INA231_CONFIG_BUS_CT+INA231_CONFIG_AVG)

#define INA231_CONST				0.00512f  /* is an internal fixed value used to ensure scaling is maintained properly  */
#define INA231_VBUSSCALE			0.00125f  /* LSB of bus voltage is 1.25 mV  */
#define INA231_VSHUNTSCALE			0.0000025f /* LSB of shunt voltage is 2.5 uV  */

/* From SCH-M00041 REVB */
#define VOXLPM_INA231_VBAT_SHUNT		0.0005f   /* VBAT shunt is 500 micro-ohm */
#define VOXLPM_INA231_VREG_SHUNT		0.005f    /* VREG output shunt is 5 milli-ohm */
#define VOXLPM_INA231_VBAT_MAX_AMPS		90.0f     /* 90.0 Amps max through VBAT sense resistor */
#define VOXLPM_INA231_VREG_MAX_AMPS		6.0f      /* 6.0 Amps max through VREG sense resistor */

/* ina231.pdf section 8.5 */
#define VOXLPM_INA231_VBAT_I_LSB		(VOXLPM_INA231_VBAT_MAX_AMPS/32768.0f)
#define VOXLPM_INA231_VREG_I_LSB		(VOXLPM_INA231_VREG_MAX_AMPS/32768.0f)

#define POWER_I2C I2C1 // Comment this out to disable I2C
#define POWER_I2C_PIN_SDA GPIO_PIN(GPIOB, 9) // PB.09
#define POWER_I2C_SDA_AF GPIO_AF4
#define POWER_I2C_PIN_SCL GPIO_PIN(GPIOB, 8) // PB.08
#define POWER_I2C_SCL_AF GPIO_AF4
#define POWER_I2C_TIMING_REGISTER 0x20707DBDU // 100KHz clock
#define POWER_I2C_SLAVE_ADDRESS VOXLPM_INA231_ADDR_VBATT // or 0x45


// Audio
// #warning Disable audio and haptics
// #define HAPTIC_GPIO                   GPIO_PIN(GPIOC, 12) // PC.12

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
