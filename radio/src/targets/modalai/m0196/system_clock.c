/* ModalAI M0196 / Helm clock tree. */

#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_system.h"

#include "system_clock.h"

#define BOOTSTRAP __attribute__((section(".bootstrap")))
#define HSE_STARTUP_TIMEOUT 8000000U

extern uint32_t _reboot_cmd;

BOOTSTRAP void SystemClock_Config(void)
{
  LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while (LL_PWR_IsActiveFlag_VOS() == 0) {
  }

  LL_RCC_HSE_Enable();
  uint32_t timeout = HSE_STARTUP_TIMEOUT;
  while (LL_RCC_HSE_IsReady() != 1 && --timeout != 0) {
  }

  if (LL_RCC_HSE_IsReady() != 1) {
    // Do not run EdgeTX with its 400 MHz timing constants while still on HSI.
    // Retain the reason across reset and wait for the oscillator to recover.
    _reboot_cmd = HELM_REBOOT_CMD_HSE_FAILURE;
    while (LL_RCC_HSE_IsReady() != 1) {
    }
    NVIC_SystemReset();
  }

  // VOS1 at a 200 MHz AXI clock requires two wait states and programming
  // delay setting 2. Configure both before increasing the clock frequency.
  const uint32_t flash_config =
      LL_FLASH_LATENCY_2 | FLASH_ACR_WRHIGHFREQ_1;
  MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY | FLASH_ACR_WRHIGHFREQ,
             flash_config);
  while (READ_BIT(FLASH->ACR,
                  FLASH_ACR_LATENCY | FLASH_ACR_WRHIGHFREQ) != flash_config) {
  }

  // PLL1: 16 MHz / 2 * 100 = 800 MHz VCO; P = 400 MHz SYSCLK.
  LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE);
  LL_RCC_PLL1P_Enable();
  LL_RCC_PLL1FRACN_Disable();
  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL1_SetM(2);
  LL_RCC_PLL1_SetN(100);
  LL_RCC_PLL1_SetP(2);
  LL_RCC_PLL1_SetQ(16);
  LL_RCC_PLL1_SetR(2);
  LL_RCC_PLL1_Enable();
  while (LL_RCC_PLL1_IsReady() != 1) {
  }

  LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_2);
  LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1) {
  }

  // PLL2: 16 MHz / 8 * 100 = 200 MHz VCO.
  // P = 100 MHz for ADC and SPI1/2/3; R = 200 MHz for SDMMC.
  LL_RCC_PLL2P_Enable();
  LL_RCC_PLL2R_Enable();
  LL_RCC_PLL2FRACN_Disable();
  LL_RCC_PLL2_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_2_4);
  LL_RCC_PLL2_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL2_SetM(8);
  LL_RCC_PLL2_SetN(100);
  LL_RCC_PLL2_SetP(2);
  LL_RCC_PLL2_SetQ(2);
  LL_RCC_PLL2_SetR(1);
  LL_RCC_PLL2_Enable();
  while (LL_RCC_PLL2_IsReady() != 1) {
  }

  // PLL3: 16 MHz / 4 * 96 = 384 MHz VCO; Q = 48 MHz for both USB cores.
  LL_RCC_PLL3Q_Enable();
  LL_RCC_PLL3FRACN_Disable();
  LL_RCC_PLL3_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_4_8);
  LL_RCC_PLL3_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL3_SetM(4);
  LL_RCC_PLL3_SetN(96);
  LL_RCC_PLL3_SetP(2);
  LL_RCC_PLL3_SetQ(8);
  LL_RCC_PLL3_SetR(2);
  LL_RCC_PLL3_Enable();
  while (LL_RCC_PLL3_IsReady() != 1) {
  }

  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLL3Q);
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PLL2P);
  LL_RCC_SetSDMMCClockSource(LL_RCC_SDMMC_CLKSOURCE_PLL2R);
  LL_RCC_SetSPIClockSource(LL_RCC_SPI123_CLKSOURCE_PLL2P);
  LL_RCC_SetSPIClockSource(LL_RCC_SPI45_CLKSOURCE_PCLK2);
  LL_RCC_SetSPIClockSource(LL_RCC_SPI6_CLKSOURCE_PCLK4);

  // CSS automatically switches SYSCLK to HSI and raises NMI on HSE failure.
  LL_RCC_HSE_EnableCSS();
}
