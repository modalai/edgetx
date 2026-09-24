/*
 * HELM_BASIC population and input map.
 *
 * Keep this file as a flat list of revision-specific definitions. The common
 * M0196 HAL derives bus availability and EdgeTX hardware definitions from it.
 */

#pragma once

#define HELM_HAS_SOFT_POWER
#define HELM_HAS_RGB_LED
#define HELM_HAS_OB_EEPROM
#define HELM_HAS_ANALOG_GIMBALS

#define HELM_HAS_BS_R1
#define HELM_HAS_BS_R2
#define HELM_HAS_BS_R3
#define HELM_HAS_BS_R4
#define HELM_HAS_BS_R5
#define HELM_HAS_BS_L1
#define HELM_HAS_BS_L2
#define HELM_HAS_BS_L3
#define HELM_HAS_BS_L4
#define HELM_HAS_BS_L5

#define HELM_HAS_FIVE_WAY_UPPER_LEFT
#define HELM_HAS_FIVE_WAY_UPPER_RIGHT
#define HELM_HAS_FIVE_WAY_LOWER_LEFT
#define HELM_HAS_FIVE_WAY_LOWER_RIGHT
#define HELM_HAS_QUICK_BUTTONS

#define HELM_NUM_FUNCTION_SWITCHES 5
#define HELM_NUM_FUNCTION_GROUPS 3
#define HELM_FUNCTION_SWITCH_1 MENU_1
#define HELM_FUNCTION_SWITCH_2 MENU_2
#define HELM_FUNCTION_SWITCH_3 MENU_3
#define HELM_FUNCTION_SWITCH_4 MENU_4
#define HELM_FUNCTION_SWITCH_5 MENU_5

#define HELM_HAS_INTERNAL_MODULE
#define HELM_HAS_INTERNAL_FAN
#define HELM_HAS_HAPTICS
#define HELM_HAS_SD_CARD

// Test the three unmapped five-way contacts and the soft-power button.
#define FACTORY_TEST_EXTRA_INPUTS
#define FACTORY_TEST_POWER_BUTTON

// Right-side EdgeTX switches.
#define HELM_SWITCH_A BS_R2_A
#define HELM_SWITCH_B_HIGH BS_R1_A
#define HELM_SWITCH_B_LOW BS_R1_B
#define HELM_SWITCH_C_HIGH BS_R3_A
#define HELM_SWITCH_C_LOW BS_R3_B
#define HELM_SWITCH_D_HIGH BS_R5_A
#define HELM_SWITCH_D_LOW BS_R5_B
#define HELM_SWITCH_E_HIGH BS_R4_A
#define HELM_SWITCH_E_LOW BS_R4_B

// Left-side EdgeTX switches.
#define HELM_SWITCH_F BS_L2_A
#define HELM_SWITCH_G BS_L1_A
#define HELM_SWITCH_H_HIGH BS_L3_A
#define HELM_SWITCH_H_LOW BS_L3_B
#define HELM_SWITCH_I_HIGH LOW_L_B
#define HELM_SWITCH_I_LOW LOW_L_C
#define HELM_SWITCH_J_HIGH BS_L5_A
#define HELM_SWITCH_J_LOW BS_L5_B
#define HELM_SWITCH_K_HIGH BS_L4_A
#define HELM_SWITCH_K_LOW BS_L4_B

// Lower five-way switch axes.
#define HELM_SWITCH_L_HIGH LOW_L_D
#define HELM_SWITCH_L_LOW LOW_L_A
#define HELM_SWITCH_M_HIGH LOW_R_A
#define HELM_SWITCH_M_LOW LOW_R_D

// Hide the lower five-way axes from the compact main-view switch display.
#define MAIN_VIEW_HIDDEN_SWITCHES \
  ((1u << 8) | (1u << 11) | (1u << 12))

// Upper-left: A=left, B=up, C=down, D=right, E=center.
// Upper-right: A=right, B=down, C=up, D=left, E=center.
#define HELM_KEY_EXIT UPR_L_E
#define HELM_KEY_MENU UPR_L_B
#define HELM_KEY_SYS UPR_L_A
#define HELM_KEY_MDL UPR_L_D
#define HELM_KEY_TELE UPR_L_C
#define HELM_KEY_PAGEUP UPR_R_A
#define HELM_KEY_PAGEDN UPR_R_D
#define HELM_KEY_UP UPR_R_C
#define HELM_KEY_DOWN UPR_R_B
#define HELM_KEY_ENTER UPR_R_E

// Gimbals.
#define HELM_ADC_STICK_LH GIMBAL_LH
#define HELM_ADC_STICK_LV GIMBAL_LV
#define HELM_ADC_STICK_LV_INVERTED
#define HELM_ADC_STICK_RV GIMBAL_RV
#define HELM_ADC_STICK_RH GIMBAL_RH
#define HELM_ADC_STICK_RH_INVERTED
