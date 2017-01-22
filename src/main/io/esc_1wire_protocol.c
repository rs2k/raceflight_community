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
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "platform.h"
#ifdef ESC_1WIRE
#include "drivers/gpio.h"
#include "drivers/system.h"
#include "esc_1wire_protocol.h"
#include "esc_1wire_blheli.h"
uint8_t esc1WireBuf[ESC_BUF_SIZE];
uint8_t esc1WireVerifyBuf[ESC_BUF_SIZE];
void setEscState(escHardware_t *esc, uint8_t state) {
    if (state) {
#ifdef STM32F4
        esc->gpio->BSRRL = esc->pin;
#else
        esc->gpio->BSRR = esc->pin;
#endif
    } else {
#ifdef STM32F4
        esc->gpio->BSRRH = esc->pin;
#else
        esc->gpio->BRR = esc->pin;
#endif
    }
}
uint8_t getEscState(escHardware_t *esc) {
    return !! (esc->gpio->IDR & esc->pin);
}
static void escGPIOConfig(GPIO_TypeDef *gpio, uint16_t pin, GPIO_Mode mode) {
    gpio_config_t cfg;
    cfg.pin = pin;
    cfg.mode = mode;
    cfg.speed = Speed_50MHz;
    gpioInit(gpio, &cfg);
}
void setEscInput(escHardware_t *esc) {
    escGPIOConfig(esc->gpio, esc->pin, Mode_IPU);
}
void setEscOutput(escHardware_t *esc) {
    escGPIOConfig(esc->gpio, esc->pin, Mode_Out_PP);
}
void setEscReset(escHardware_t *esc) {
    escGPIOConfig(esc->gpio, esc->pin, Mode_AF_PP);
}
static uint16_t signaturesAtmel[] = {0x9307, 0x930A, 0x930F, 0x940B, 0};
static uint16_t signaturesSilabs[] = {0xF310, 0xF330, 0xF410, 0xF390, 0xF850, 0xE8B1, 0xE8B2, 0};
static bool signatureMatch(uint16_t signature, uint16_t *list) {
    for(; *list; list++) {
        if (signature == *list) {
            return true;
        }
    }
    return false;
}
uint8_t initializeBLHeliDevice(escHardware_t *esc, escDeviceInfo_t *device) {
#if 0
    if (Stk_ConnectEx(pDeviceInfo) && signatureMatch(pDeviceInfo->signature, signaturesAtmel)) {
        escHardware->deviceInfo.bootloaderMode = imSK;
        return true;
    }
#endif
    if (signatureMatch(device->signature, signaturesSilabs)) {
        device->bootloaderMode = BLHeli_Silabs;
        device->protocol = &BLHeliSiLabsProtocol;
    } else if (signatureMatch(device->signature, signaturesAtmel)) {
        device->bootloaderMode = BLHeli_Atmel;
        device->protocol = &BLHeliAtmelProtocol;
    } else {
        device->bootloaderMode = 0;
        device->protocol = NULL;
        return false;
    }
    ioMem_t ioMem = {
        .addr = 0,
        .len = ESC_BUF_SIZE,
        .data = esc1WireBuf,
    };
    if (!device->protocol->readEEprom(esc, &ioMem)) {
        return false;
    }
    device->layout = getBLHeliEEpromLayout(&ioMem);
    device->version = getBLHeliVersion(&ioMem);
    getBLHeliEscName(&ioMem, device->name);
    if (device->layout == NULL) {
        return false;
    } else {
        return true;
    }
}
#endif
