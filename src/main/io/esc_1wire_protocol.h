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
 */

#pragma once

#ifdef ESC_1WIRE


#define ESC_BUF_SIZE 256

typedef struct BLHeli_EEprom BLHeli_EEprom_t;

typedef enum {
    BLHeli_Silabs = 1,
    BLHeli_Atmel,
    SimonK_Atmel
} escBootloader_e;

typedef struct {
    GPIO_TypeDef *gpio;
    uint16_t pin;
} escHardware_t;

typedef struct {
    uint16_t len;
    uint16_t addr;
    uint8_t *data;
} ioMem_t;

typedef struct {
    bool (*pollReadReady)(escHardware_t*, uint32_t timeout);
    bool (*readFlash)(escHardware_t*, ioMem_t*);
    bool (*writeFlash)(escHardware_t*, ioMem_t*);
    bool (*readEEprom)(escHardware_t*, ioMem_t*);
    bool (*writeEEprom)(escHardware_t*, ioMem_t*);
    bool (*pageErase)(escHardware_t*, ioMem_t*);
    bool (*eepromErase)(escHardware_t*, ioMem_t*);
} esc1WireProtocol_t;

typedef struct {
    uint16_t signature;                 
    uint8_t  signature2;                
    escBootloader_e bootloaderMode;     
    const esc1WireProtocol_t *protocol; 
    const BLHeli_EEprom_t *layout;      
    char name[16];                
    uint16_t version;                   
} escDeviceInfo_t;

uint8_t initializeBLHeliDevice(escHardware_t *esc, escDeviceInfo_t *device);


#define NO_CMD 0xFF

void setEscState(escHardware_t *esc, uint8_t state);
uint8_t getEscState(escHardware_t *esc);

void setEscInput(escHardware_t *escHardware);
void setEscOutput(escHardware_t *escHardware);
void setEscReset(escHardware_t *escHardware);

extern uint8_t esc1WireBuf[ESC_BUF_SIZE];
extern uint8_t esc1WireVerifyBuf[ESC_BUF_SIZE];

#endif
