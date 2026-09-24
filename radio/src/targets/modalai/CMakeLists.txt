option(SHUTDOWN_CONFIRMATION "Shutdown confirmation" OFF)
option(PXX1 "PXX1 protocol support" OFF)
option(PXX2 "PXX2 protocol support" OFF)
option(AFHDS3 "AFHDS3 TX Module" OFF)
option(GHOST "Ghost TX Module" ON)
option(INTERNAL_MODULE_PPM "Support for PPM internal module" OFF)
option(AUTOUPDATE "Auto update internal chips from SD" OFF)
option(BIND_KEY "Enable bind button" OFF)
option(BLUETOOTH "FrSky BT module support" OFF)
option(HELM_DIAGNOSTICS "Enable M0196 bring-up diagnostics over SEGGER RTT" OFF)

set(USE_RTC_CLOCK NO)
set(RTC_BACKUP_RAM NO)
set(TARGET_DIR modalai)

set(HSE_VALUE 16000000)
set(PPM_LIMITS_SYMETRICAL YES)

add_definitions(-DPCBFRSKY -DPCBTARANIS -DPCBMODALAI)
add_definitions(-DMANUFACTURER_MODALAI)

# Make malloc() thread-safe
add_definitions(-DTHREADSAFE_MALLOC)

# Default to telemetry view on boot
add_definitions(-DDEFAULT_VIEW_TELEMETRY)

if(PCB STREQUAL M207)
  set(MODAL_BOARD m0207)
  set(FACTORY_RESET YES)
  # M207 sets HARDWARE_EXTERNAL_MODULE NO and drives a CRSF internal module,
  # so the Ghost and Multi TX protocols are dead weight. They are also what
  # made the factory reset menu not fit: 2.12.4 plus the helm features leaves
  # RAM_D1 about 11 KB short, and dropping these frees ~27 KB. M196 gets the
  # same treatment from its own build flags.
  set(GHOST NO)
  set(MULTIMODULE NO)
  set(INTERNAL_MODULE_MULTI NO)
  set(PWR_BUTTON "SWITCH" CACHE STRING "Pwr button type (PRESS/SWITCH)")
  #set(HAPTIC YES)
  set(GUI_DIR 128x64)
  set(SDCARD YES)
  set(STORAGE_MODELSLIST NO)
  set(BITMAPS_DIR 128x64)
  set(BITMAPS_TARGET 9x_bitmaps)
  set(FONTS_TARGET 9x_fonts_1bit)
  set(LCD_DRIVER lcd_driver_spi.cpp)
  set(HARDWARE_EXTERNAL_MODULE NO)
  set(INTERNAL_MODULE_CRSF YES)
  add_definitions(-DHARDWARE_INTERNAL_MODULE)
  add_definitions(-DINTERNAL_MODULE_CRSF)
  set(STATUS_LEDS YES)
  add_definitions(-DPCBM207 -DSOFTWARE_VOLUME -DFACTORY_RESET)
  add_definitions(-DPWR_BUTTON_${PWR_BUTTON})
  if(PCBREV STREQUAL MODAL_ZORRO)
    set(DEFAULT_INTERNAL_MODULE CROSSFIRE CACHE STRING "Default internal module")
    set(INTERNAL_MODULE_SERIAL YES)
    set(MODULE_SIZE_SML YES)
    set(FLAVOUR modal_zorro)
    set(CPU_TYPE_FULL STM32H753II)
    set(ROTARY_ENCODER YES)
    set(USB_CHARGER NO)
    set(IMU NO)
    set(BLUETOOTH NO)
    #if (NOT BLUETOOTH)
    #  set(AUX_SERIAL ON)
    #endif()
    add_definitions(-DRADIO_MODAL)
    set(ENABLE_SERIAL_PASSTHROUGH ON CACHE BOOL "Enable serial passthrough")
    # LCD controller. The Zorro Blue ships with an ST7565 panel, which is the
    # default and must stay the default; ST7567 selects the init sequence for
    # the PE12864WRF-055H22Q_005 bench panel.
    #
    # This is a cache variable, and the superbuild only forwards -D arguments
    # from the invocation that set them (see the "specified on the command line"
    # filter in the top-level CMakeLists.txt). So once the arm-none-eabi
    # sub-project has been configured with a value it keeps it, and a later plain
    # `cmake .` will not clear it. Changing this needs both:
    #     cmake -DLCD_IC=ST7565 .
    #     make arm-none-eabi-configure
    # Getting it wrong is nearly invisible -- the two init sequences differ in
    # SEG/COM scan direction, so the only symptom is a 180-degree rotated screen.
    # Hence the status message below: always check it against the panel you have.
    set(LCD_IC "ST7565" CACHE STRING "LCD controller IC (ST7565 or ST7567)")
    set_property(CACHE LCD_IC PROPERTY STRINGS ST7565 ST7567)
    if(LCD_IC STREQUAL ST7567)
      add_definitions(-DLCD_IC_ST7567)
      message(STATUS "LCD controller: ST7567 (PE12864WRF bench panel, NOT stock)")
    elseif(LCD_IC STREQUAL ST7565)
      message(STATUS "LCD controller: ST7565 (stock Zorro Blue panel)")
    else()
      message(FATAL_ERROR
        "LCD_IC must be ST7565 or ST7567, got '${LCD_IC}'. "
        "A misspelled value used to fall back to ST7565 silently.")
    endif()
  endif()
elseif(PCB STREQUAL M196)
  if(NOT PCBREV STREQUAL HELM_BASIC)
    message(FATAL_ERROR "Unsupported M196 revision '${PCBREV}'")
  endif()

  set(MODAL_BOARD m0196)
  set(FACTORY_RESET YES)
  set(USE_RTC_CLOCK YES)
  set(GUI_DIR 128x64)
  set(SDCARD YES)
  set(STORAGE_MODELSLIST NO)
  set(BITMAPS_DIR 128x64)
  set(BITMAPS_TARGET 9x_bitmaps)
  set(FONTS_TARGET 9x_fonts_1bit)
  set(LCD_DRIVER lcd_driver.cpp)
  set(HARDWARE_EXTERNAL_MODULE NO)
  set(INTERNAL_MODULE_CRSF YES)
  set(DEFAULT_INTERNAL_MODULE CROSSFIRE CACHE STRING "Default internal module" FORCE)
  set(INTERNAL_MODULE_SERIAL YES)
  set(MODULE_SIZE_SML YES)
  set(FLAVOUR helm_basic)
  set(CPU_TYPE_FULL STM32H753II)
  set(SEGGER_RTT_SECTION ".dram")
  set(ROTARY_ENCODER NO)
  set(USB_CHARGER NO)
  set(IMU NO)
  set(BLUETOOTH NO)
  set(HAPTIC YES)
  set(FUNCTION_SWITCHES ON)
  set(STATUS_LEDS YES)
  add_definitions(
    -DPCBM196
    -DRADIO_HELM
    -DFACTORY_RESET
    -DHELM_REVISION_HELM_BASIC
    -DHELM_CMAKE_HAS_SD_CARD
    -DHARDWARE_INTERNAL_MODULE
    -DINTERNAL_MODULE_CRSF
    -DHAPTIC
    -DFUNCTION_SWITCHES
    -DHELM_CMAKE_HAS_FUNCTION_SWITCHES
    -DHELM_FACTORY_READ_ONLY_STORAGE
    -DHELM_HOST_KEY_INJECTION
    -DSOFTWARE_VOLUME
  )
  set(ENABLE_SERIAL_PASSTHROUGH ON CACHE BOOL "Enable serial passthrough" FORCE)

  if(HELM_DIAGNOSTICS)
    add_definitions(-DHELM_DIAGNOSTICS)
    set(DEBUG RTT CACHE STRING "Debug transport" FORCE)
    set(DEBUG_SEGGER_RTT ON CACHE BOOL "Enable SEGGER RTT" FORCE)
  endif()
else()
  message(FATAL_ERROR "Unsupported ModalAI PCB '${PCB}'")
endif()

message("CPU_TYPE_FULL = ${CPU_TYPE_FULL}")
if (CPU_TYPE_FULL STREQUAL STM32H753II)
  set(CPU_TYPE STM32H7)
  add_definitions(-DSTM32H753xx)
  set(TARGET_LINKER_DIR stm32h753)
else()
  message(FATAL_ERROR "Unknown CPU_TYPE_FULL" )
endif()


if(ENABLE_SERIAL_PASSTHROUGH)
 set(CLI ON "Enable CLI")
endif()

if(INTERNAL_MODULE_SERIAL)

  # defines default internal modules
  set(INTERNAL_MODULES MULTI;CRSF CACHE STRING "Internal modules")

  add_definitions(-DINTERNAL_MODULE_SERIAL)

  endif()

set(TARGET_SRC_DIR targets/${TARGET_DIR}/${MODAL_BOARD})
set(TARGET_HAL_FILE ${TARGET_SRC_DIR}/hal.h)
if(PCB STREQUAL M196)
  set(TARGET_HAL_DEPS
    ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_SRC_DIR}/helm_basic.h
    ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_SRC_DIR}/pins.h
  )
endif()
include_directories(${TARGET_SRC_DIR} targets/${TARGET_DIR})

set(BOARD_COMMON_SRC
  ${TARGET_SRC_DIR}/board.cpp
  ${TARGET_SRC_DIR}/led_driver.cpp
  targets/${TARGET_DIR}/backlight_driver.cpp
  ${TARGET_SRC_DIR}/${LCD_DRIVER}
  targets/common/arm/stm32/abnormal_reboot.cpp
  targets/common/arm/stm32/delays_driver.cpp
  targets/common/arm/stm32/flash_driver.cpp
  targets/common/arm/stm32/pwr_driver.cpp
  targets/common/arm/stm32/rtc_driver.cpp
  targets/common/arm/stm32/watchdog_driver.cpp
)

if(HAPTIC)
  list(APPEND BOARD_COMMON_SRC ${TARGET_SRC_DIR}/haptic_driver.cpp)
endif()

if(PCB STREQUAL M207)
  list(APPEND BOARD_COMMON_SRC ${TARGET_SRC_DIR}/voxlpm_i2c_driver.cpp)
elseif(PCB STREQUAL M196)
  list(APPEND BOARD_COMMON_SRC ${TARGET_SRC_DIR}/voxlpm_i2c_driver.cpp)
  list(APPEND BOARD_COMMON_SRC ${TARGET_SRC_DIR}/rtt_key_injection.cpp)
  if(HELM_DIAGNOSTICS)
    list(APPEND BOARD_COMMON_SRC ${TARGET_SRC_DIR}/diagnostics.cpp)
  endif()
endif()

if(ROTARY_ENCODER)
  list(APPEND BOARD_COMMON_SRC
    targets/common/arm/stm32/rotary_encoder_driver.cpp
  )
endif()

if(BLUETOOTH)
  list(APPEND BOARD_COMMON_SRC
    targets/common/arm/stm32/bluetooth_driver.cpp
  )
endif()

# Bootloader board library
add_library(board_bl OBJECT EXCLUDE_FROM_ALL
  ${BOARD_COMMON_SRC}
  ${TARGET_SRC_DIR}/system_clock.c
  ${RADIO_SRC_DIR}/gui/common/stdlcd/boot_menu.cpp
)
add_dependencies(board_bl ${BITMAPS_TARGET})
set(BOOTLOADER_SRC ${BOOTLOADER_SRC} $<TARGET_OBJECTS:board_bl>)

# Firmware board library
add_library(board OBJECT EXCLUDE_FROM_ALL
  ${BOARD_COMMON_SRC}
  targets/${TARGET_DIR}/audio_driver.cpp
  targets/common/arm/stm32/delays_driver.cpp
  targets/common/arm/stm32/mixer_scheduler_driver.cpp
  targets/common/arm/stm32/module_timer_driver.cpp
  targets/common/arm/stm32/stm32_pulse_driver.cpp
  targets/common/arm/stm32/stm32_softserial_driver.cpp
  targets/common/arm/stm32/stm32_switch_driver.cpp
  targets/common/arm/stm32/stm32_ws2812.cpp
)
if(PCB STREQUAL M196)
  target_sources(board PRIVATE ${TARGET_SRC_DIR}/eeprom_93aa66.cpp)
endif()
set(FIRMWARE_SRC ${FIRMWARE_SRC} $<TARGET_OBJECTS:board>)

if(BIND_KEY)
  add_definitions(-DBIND_KEY)
  target_sources(board PRIVATE ${TARGET_SRC_DIR}/bind_button_driver.cpp)
endif()

if(USB_CHARGE_LED)
  add_definitions(-DUSB_CHARGE_LED)
endif()

if(USB_CHARGER)
  add_definitions(-DUSB_CHARGER)
  target_sources(board PRIVATE ${TARGET_SRC_DIR}/usb_charger_driver.cpp)
endif()

if(IMU)
  target_sources(board PRIVATE targets/common/arm/stm32/lsm6ds_driver.cpp)
endif()

if(AUTOUPDATE)
  add_definitions(-DAUTOUPDATE)
endif()

if(USE_RTC_CLOCK)
  add_definitions(-DRTCLOCK)
endif()

set(SRC ${SRC}
  io/frsky_firmware_update.cpp
  io/multi_firmware_update.cpp
)

if(FACTORY_RESET)
  set(FACTORY_SD_DIR "${CMAKE_SOURCE_DIR}/resources/sd-card/${MODAL_BOARD}")
  if(NOT EXISTS "${FACTORY_SD_DIR}")
    message(FATAL_ERROR "Factory SD resources are missing: ${FACTORY_SD_DIR}")
  endif()

  file(GLOB_RECURSE FACTORY_SD_INPUTS CONFIGURE_DEPENDS
    LIST_DIRECTORIES false "${FACTORY_SD_DIR}/*")
  set(FACTORY_SD_IMAGE "${CMAKE_CURRENT_BINARY_DIR}/factory_sd_image.cpp")
  if(PCB STREQUAL M196)
    set(FACTORY_SD_BINARY "${CMAKE_CURRENT_BINARY_DIR}/factory_sd.img")
    add_custom_command(
      OUTPUT "${FACTORY_SD_IMAGE}" "${FACTORY_SD_BINARY}"
      COMMAND ${PYTHON_EXECUTABLE}
        "${RADIO_DIRECTORY}/util/pack_factory_sd.py"
        --input "${FACTORY_SD_DIR}"
        --output "${FACTORY_SD_IMAGE}"
        --image "${FACTORY_SD_BINARY}"
      DEPENDS
        "${RADIO_DIRECTORY}/util/pack_factory_sd.py"
        ${FACTORY_SD_INPUTS}
      COMMENT "Building ${MODAL_BOARD} factory FAT image"
      VERBATIM
    )
  else()
    add_custom_command(
      OUTPUT "${FACTORY_SD_IMAGE}"
      COMMAND ${PYTHON_EXECUTABLE}
        "${RADIO_DIRECTORY}/util/pack_factory_sd_archive.py"
        --input "${FACTORY_SD_DIR}"
        --output "${FACTORY_SD_IMAGE}"
      DEPENDS
        "${RADIO_DIRECTORY}/util/pack_factory_sd_archive.py"
        ${FACTORY_SD_INPUTS}
      COMMENT "Packing ${MODAL_BOARD} factory SD archive"
      VERBATIM
    )
  endif()
  set_source_files_properties("${FACTORY_SD_IMAGE}" PROPERTIES GENERATED TRUE)
  if(PCB STREQUAL M196)
    list(APPEND SRC
      factory_reset.cpp
      storage/factory_volume.cpp
      storage/helm_device_settings.cpp
      "${FACTORY_SD_IMAGE}"
    )
  else()
    list(APPEND SRC factory_reset_archive.cpp "${FACTORY_SD_IMAGE}")
  endif()
  list(APPEND GUI_SRC radio_factory_reset.cpp)
endif()

set(GUI_SRC ${GUI_SRC}
  model_input_edit.cpp
  model_mix_edit.cpp
  model_display.cpp
  radio_diagkeys.cpp
  radio_diaganas.cpp
  view_channels.cpp
  view_telemetry.cpp
  view_about.cpp
  bmp.cpp
)

# Zorro Blue bench tool: drives the lcdSet*() knobs that only the M0207 ST7565
# SPI driver exposes.
if(PCB STREQUAL M207)
  set(GUI_SRC ${GUI_SRC} view_lcd_tuning.cpp)
endif()

set(FIRMWARE_SRC
  ${FIRMWARE_SRC}
  bootloader/loadboot.cpp
)

set(RADIO_DEPENDENCIES ${RADIO_DEPENDENCIES} ${BITMAPS_TARGET})
