#include "diagnostics.h"

#include "board.h"
#include "debug.h"

#if !defined(BOOT)
#include "hal/adc_driver.h"
#include "hal/key_driver.h"
#include "hal/switch_driver.h"
#endif

void helmDiagnosticsBootloader()
{
  TRACE("HELM: bootloader board initialization");
}

void helmDiagnosticsInit()
{
  TRACE("HELM: application board initialization");
  TRACE("HELM: CPU=%u Hz, APB1=%u Hz, APB2=%u Hz", CPU_FREQ,
        PERI1_FREQUENCY, PERI2_FREQUENCY);
  TRACE("HELM clocks: AHB=%u ADC=%u SDMMC=%u USB=%u", AHB_FREQUENCY,
        ADC_FREQUENCY, SDMMC_KERNEL_FREQUENCY, USB_KERNEL_FREQUENCY);
  TRACE("HELM LCD: ST7567 SPI=%u Hz SEG=0x%02x COM=0x%02x column=%u",
        LCD_SPI_FREQUENCY, LCD_SEG_DIRECTION_COMMAND,
        LCD_COM_DIRECTION_COMMAND, LCD_COLUMN_OFFSET);
  TRACE("HELM: internal RF power remains safely disabled");
}

void helmDiagnosticsPer5ms()
{
#if !defined(BOOT)
  static uint16_t ticks;
  if (++ticks < 200) return;
  ticks = 0;

  TRACE("HELM ADC: %u %u %u %u %u %u %u %u", anaIn(0), anaIn(1),
        anaIn(2), anaIn(3), anaIn(4), anaIn(5), anaIn(6), anaIn(7));
  TRACE("HELM keys=0x%08x trims=0x%08x switches=%u", readKeys(),
        readTrims(), switchGetMaxSwitches());
  TRACE("HELM LCD: frames=%u dma_errors=%u", lcdGetRefreshCount(),
        lcdGetDmaErrorCount());
#endif
}
