/* 
 * This file is part of RaceFlight. 
 * 
 * RaceFlight is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, or 
 * (at your option) any later version. 
 * 
 * RaceFlight is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 * 
 * You should have received a copy of the GNU General Public License 
 * along with RaceFlight.  If not, see <http://www.gnu.org/licenses/>.
 * You should have received a copy of the GNU General Public License 
 * along with RaceFlight.  If not, see <http://www.gnu.org/licenses/>.
 */ 
#pragma once 
       
#ifdef ESC_1WIRE
#define BLHELI_EEPROM_HEAD 3
#include "io/esc_1wire_protocol.h"
typedef struct BLHeli_EEprom {
    uint8_t BL_GOV_P_GAIN;
    uint8_t BL_GOV_I_GAIN;
    uint8_t BL_GOV_MODE;
    uint8_t BL_MOT_GAIN;
    uint8_t BL_STARTUP_PWR;
    uint8_t BL_PWM_FREQ;
    uint8_t BL_DIRECTION;
    uint8_t BL_INPUT_POL;
    uint8_t BL_INIT_L;
    uint8_t BL_INIT_H;
    uint8_t BL_ENABLE_TX;
    uint8_t BL_COMM_TIMING;
    uint8_t BL_PPM_MIN_THROTLE;
    uint8_t BL_PPM_MAX_THROTLE;
    uint8_t BL_BEEP_STRENGTH;
    uint8_t BL_BEACON_STRENGTH;
    uint8_t BL_BEACON_DELAY;
    uint8_t BL_DEMAG_COMP;
    uint8_t BL_BEC_VOLTAGE_HIGH;
    uint8_t BL_PPM_CENTER;
    uint8_t BL_TEMP_PROTECTION;
    uint8_t BL_ENABLE_POWER_PROT;
    uint8_t BL_ENABLE_PWM_INPUT;
    uint8_t BL_PWM_DITHER;
    uint8_t BL_BRAKE_ON_STOP;
    uint8_t BL_LED_CONTROL;
} BLHeli_EEprom_t;
extern const esc1WireProtocol_t BLHeliAtmelProtocol;
extern const esc1WireProtocol_t BLHeliSiLabsProtocol;
uint8_t connectBLHeliDevice(escHardware_t *escHardware, escDeviceInfo_t *escDevice);
uint16_t getBLHeliVersion(ioMem_t *ioMem);
const BLHeli_EEprom_t* getBLHeliEEpromLayout(ioMem_t *ioMem);
void getBLHeliEscName(ioMem_t* ioMem, char *output);
#endif
