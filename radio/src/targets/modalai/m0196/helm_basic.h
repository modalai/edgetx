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

#define HELM_HAS_INTERNAL_MODULE
#define HELM_HAS_INTERNAL_FAN
#define HELM_HAS_HAPTICS
#define HELM_HAS_SD_CARD

// EdgeTX switches, ordered from the upper pair toward the lower pair.
#define HELM_SWITCH_A_HIGH BS_R1_A
#define HELM_SWITCH_A_LOW BS_R1_B
#define HELM_SWITCH_B_HIGH BS_L1_A
#define HELM_SWITCH_B_LOW BS_L1_B
#define HELM_SWITCH_C_HIGH BS_R2_A
#define HELM_SWITCH_C_LOW BS_R2_B
#define HELM_SWITCH_D_HIGH BS_L2_A
#define HELM_SWITCH_D_LOW BS_L2_B
#define HELM_SWITCH_E_HIGH BS_R3_A
#define HELM_SWITCH_E_LOW BS_R3_B
#define HELM_SWITCH_F_HIGH BS_L3_A
#define HELM_SWITCH_F_LOW BS_L3_B
#define HELM_SWITCH_G_HIGH BS_R4_A
#define HELM_SWITCH_G_LOW BS_R4_B
#define HELM_SWITCH_H_HIGH BS_L4_A
#define HELM_SWITCH_H_LOW BS_L4_B
#define HELM_SWITCH_I_HIGH BS_R5_A
#define HELM_SWITCH_I_LOW BS_R5_B
#define HELM_SWITCH_J_HIGH BS_L5_A
#define HELM_SWITCH_J_LOW BS_L5_B

// Upper five-way switches. Contact order is A=up, B=right, C=left,
// D=down, and E=center for the installed JS1400 orientation.
#define HELM_KEY_EXIT UPR_L_E
#define HELM_KEY_SYS UPR_L_C
#define HELM_KEY_MDL UPR_L_B
#define HELM_KEY_TELE UPR_L_D
#define HELM_KEY_PAGEUP UPR_R_C
#define HELM_KEY_PAGEDN UPR_R_B
#define HELM_KEY_UP UPR_R_A
#define HELM_KEY_DOWN UPR_R_D
#define HELM_KEY_ENTER UPR_R_E

//Gimbals
#define HELM_ADC_STICK_LH GIMBAL_LH
#define HELM_ADC_STICK_LV GIMBAL_LV
#define HELM_ADC_STICK_RV GIMBAL_RV
#define HELM_ADC_STICK_RH GIMBAL_RH