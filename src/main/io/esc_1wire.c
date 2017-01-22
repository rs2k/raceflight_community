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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "platform.h"
#ifdef ESC_1WIRE
#include "build_config.h"
#include "debug.h"
#include "rx/rx.h"
#include "drivers/gpio.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_mapping.h"
#include "drivers/system.h"
#include "esc_binary.h"
#include "io/esc_1wire.h"
#include "io/esc_1wire_protocol.h"
#include "io/beeper.h"
#include "flight/mixer.h"
#include "io/esc_1wire_blheli.h"
int8_t escCount = -1;
static escHardware_t escHardware[MAX_PWM_MOTORS];
static escDeviceInfo_t escDevice[MAX_PWM_MOTORS];
static uint8_t eraseFlash(escHardware_t *esc, const esc1WireProtocol_t *proto, uint16_t addr);
static uint8_t writeBufferToFlash(escHardware_t *esc, const esc1WireProtocol_t *proto, uint16_t addr, uint8_t *data, uint16_t len);
static uint8_t readBufferFromFlash(escHardware_t *esc, const esc1WireProtocol_t *proto, uint16_t addr, uint8_t *data, uint16_t len);
int8_t esc1WireInitialize(void) {
    uint32_t wait_time;
    SKIP_GYRO = true;
    SKIP_RX = true;
    if (escCount < 0) {
        escCount = 0;
        memset(&escHardware, 0, sizeof(escHardware));
        memset(&escDevice, 0, sizeof(escDevice));
        pwmOutputConfiguration_t *pwmOutputConfiguration = pwmGetOutputConfiguration();
        pwmPortConfiguration_t *portConfigurations = pwmOutputConfiguration->portConfigurations;
        for (uint8_t i = 0; i < pwmOutputConfiguration->outputCount; i++) {
            if ((pwmOutputConfiguration->portConfigurations[i].flags & PWM_PF_MOTOR) &
                    (motor[portConfigurations[i].index] > 0)) {
                escHardware_t *esc = &escHardware[escCount];
                esc->gpio = portConfigurations[i].timerHardware->gpio;
                esc->pin = portConfigurations[i].timerHardware->pin;
                escCount++;
            }
        }
    }
    pwmDisableMotors();
    pwmShutdownPulsesForAllMotors(escCount);
    wait_time = millis() + 250;
    while (millis() < wait_time);
    for (uint8_t escIndex = 0; escIndex < escCount; escIndex++) {
        setEscInput(&escHardware[escIndex]);
        setEscState(&escHardware[escIndex], 1);
    }
    wait_time = millis() + 1500;
    while (millis() < wait_time);
    for (uint8_t escIndex = 0; escIndex < escCount; escIndex++) {
        for (int i=0; i < 6; i++) {
            if (connectBLHeliDevice(&escHardware[escIndex], &escDevice[escIndex])) {
                break;
            }
        }
    }
    for (uint8_t escIndex = 0; escIndex < escCount; escIndex++) {
        if (escDevice[escIndex].signature == 0) {
            continue;
        }
        for (int i=0; i < 6; i++) {
            if (initializeBLHeliDevice(&escHardware[escIndex], &escDevice[escIndex])) {
                break;
            }
        }
    }
    wait_time = millis() + 50;
    while (millis() < wait_time);
    return escCount;
}
void esc1WireDisconnect(void) {
    for(int escIndex = 0; escIndex < escCount; escIndex++) {
        escHardware_t *esc = &escHardware[escIndex];
        const esc1WireProtocol_t *proto = escDevice[escIndex].protocol;
        if (proto == NULL) {
            continue;
        }
        proto->disconnect(esc);
    }
    esc1WireRelease();
}
void esc1WireRelease(void) {
    uint32_t wait_time;
    for(int escIndex = 0; escIndex < escCount; escIndex++) {
        setEscReset(&escHardware[escIndex]);
        setEscState(&escHardware[escIndex], 0);
    }
    memset(&escDevice, 0, sizeof(escDevice));
    wait_time = millis() + 250;
    while (millis() < wait_time);
    pwmEnableMotors();
    pwmOutputConfiguration_t *pwmOutputConfiguration = pwmGetOutputConfiguration();
    pwmPortConfiguration_t *portConfigurations = pwmOutputConfiguration->portConfigurations;
    for (int escIndex = 0; escIndex < escCount; escIndex++) {
        pwmWriteMotor(escIndex, motor[portConfigurations[escIndex].index]);
    }
    escCount = -1;
    SKIP_GYRO = false;
    SKIP_RX = false;
}
int8_t esc1WireStatus(void) {
    return escCount;
}
int8_t esc1WireCheckMotor(uint8_t escIndex) {
    if (escCount < 0 || escIndex >= escCount) {
        return BLHELI_NOT_STARTED;
    }
    if (escDevice[escIndex].signature == 0) {
        return BLHELI_NO_CONNECTION;
    }
    if (!escDevice[escIndex].protocol) {
        return BLHELI_BAD_PROTOCOL;
    }
    if (!escDevice[escIndex].layout) {
        return BLHELI_BAD_LAYOUT;
    }
    return BLHELI_OK;
}
uint8_t esc1WireInitFlash(uint8_t escIndex) {
    escHardware_t *esc = &escHardware[escIndex];
    if (esc == NULL) {
        return false;
    }
    const esc1WireProtocol_t *proto = escDevice[escIndex].protocol;
    if (proto == NULL) {
        return false;
    }
    uint8_t attempt;
    uint8_t escData[3];
    for (attempt = 0; attempt < 5; attempt++) {
        if (!readBufferFromFlash(esc, proto, 0, escData, 3)) {
            continue;
        }
        if (escData[0] == 0x02 && escData[1] == 0x19 && escData[2] == 0xFD) {
            break;
        }
    }
    if (attempt == 5) {
        return false;
    }
    escData[0] = 0x02;
    escData[1] = 0x1C;
    escData[2] = 0x00;
    for (attempt = 0; attempt < 5; attempt++) {
        if (!eraseFlash(esc, proto, 0x200)) {
            continue;
        }
        if (!writeBufferToFlash(esc, proto, 0x200, escData, 3)) {
            continue;
        }
        break;
    }
    if (attempt == 5) {
        return false;
    }
    return true;
}
uint8_t esc1WireFlash(uint8_t escIndex, uint8_t *data, uint16_t address, uint16_t length) {
    uint16_t pageAddress = 0x200 * (address / 0x200);
    uint8_t attempt;
    escHardware_t *esc = &escHardware[escIndex];
    if (esc == NULL) {
        return false;
    }
    const esc1WireProtocol_t *proto = escDevice[escIndex].protocol;
    if (proto == NULL) {
        return false;
    }
    if (length > 0x200) {
        return false;
    }
    if (pageAddress != 0x200 * ((address + length - 1) / 0x200)) {
        return false;
    }
    for (attempt = 0; attempt < 5; attempt++) {
        if (!eraseFlash(esc, proto, pageAddress)) {
            continue;
        }
        if (length > 0x100) {
            if (!writeBufferToFlash(esc, proto, address, data, 0x100)) {
                continue;
            }
            if (!writeBufferToFlash(esc, proto, address+0x100, data+0x100, length-0x100)) {
                continue;
            }
        } else {
            if (!writeBufferToFlash(esc, proto, address, data, length)) {
                continue;
            }
        }
        break;
    }
    if (attempt == 5) {
        return false;
    }
    return true;
}
#ifdef ESC_HEX
uint16_t esc1WireGetHexVersion(const uint8_t *binary) {
    uint16_t version;
    version = (binary[0x1a00] << 8) | binary[0x1a01];
    return version;
}
uint8_t esc1WireGetHexMcuName(const uint8_t *binary, char *name, uint8_t len) {
    uint8_t index;
    const char *input = (char*)&binary[0x1a40];
    if (*(input++) != '#') {
        return false;
    }
    for (index = 0; index < len - 1 && input[index] != '#'; index++) {
        name[index] = input[index];
    }
    name[index] = '\0';
    return input[index] == '#';
}
const uint8_t* esc1WireFindHex(char *escName) {
   char newEscName[16];
   uint8_t firmwareIndex;
   const uint8_t *firmware;
    for (firmwareIndex = 0; escHexList[firmwareIndex] != NULL; firmwareIndex++) {
        firmware = (const uint8_t*)escHexList[firmwareIndex];
        if (!esc1WireGetHexMcuName(firmware, newEscName, 16)) {
            continue;
        }
        if (strcmp(escName, newEscName) == 0) {
            return firmware;
        }
    }
    return NULL;
}
#endif
static uint8_t eraseFlash(escHardware_t *esc, const esc1WireProtocol_t *proto, uint16_t addr) {
    ioMem_t ioMem = {
        .addr = addr,
        .len = 0,
        .data = NULL,
    };
    if (!proto->pageErase(esc, &ioMem)) {
        return false;
    }
    return true;
}
static uint8_t writeBufferToFlash(escHardware_t *esc, const esc1WireProtocol_t *proto, uint16_t addr, uint8_t *data, uint16_t len) {
    ioMem_t ioMem = {
        .addr = addr,
        .len = len,
        .data = data,
    };
    if (!proto->writeFlash(esc, &ioMem)) {
        return false;
    }
    ioMem.addr = addr;
    ioMem.len = len;
    ioMem.data = esc1WireVerifyBuf;
    if (!proto->readFlash(esc, &ioMem)) {
        return false;
    }
    if (memcmp(esc1WireVerifyBuf, data, len) != 0) {
        return false;
    }
    return true;
}
static uint8_t readBufferFromFlash(escHardware_t *esc, const esc1WireProtocol_t *proto, uint16_t addr, uint8_t *data, uint16_t len) {
    ioMem_t ioMem = {
        .addr = addr,
        .len = len,
        .data = data,
    };
    if (!proto->readFlash(esc, &ioMem)) {
        return false;
    }
    return true;
}
static int16_t readValueFromEEprom(escHardware_t *esc, const esc1WireProtocol_t *proto, uint8_t eepromIndex) {
    uint8_t attempt;
    char name[16];
    if (eepromIndex == NO_CMD) {
        return -1;
    }
    ioMem_t ioMem = {
        .addr = 0,
        .len = ESC_BUF_SIZE,
        .data = esc1WireBuf,
    };
    for (attempt = 0; attempt < 5; attempt++) {
        if (!proto->readEEprom(esc, &ioMem)) {
            continue;
        }
        if (esc1WireBuf[0] == 0xFF || esc1WireBuf[0] == 0 ||
                esc1WireBuf[1] == 0xFF || esc1WireBuf[1] == 0 ||
                esc1WireBuf[2] == 0xFF || esc1WireBuf[2] == 0) {
            continue;
        }
        getBLHeliEscName(&ioMem, name);
        if (name[0] == '\0') {
            continue;
        }
        break;
    }
    if (attempt == 5) {
        return -1;
    }
    return esc1WireBuf[BLHELI_EEPROM_HEAD + eepromIndex];
}
static uint8_t writeValueToEEprom(escHardware_t *esc, const esc1WireProtocol_t *proto, uint8_t eepromIndex, uint8_t value) {
    uint8_t attempt;
    char name[16];
    if (eepromIndex == NO_CMD) {
        return -1;
    }
    ioMem_t ioMem = {
        .addr = 0,
        .len = ESC_BUF_SIZE,
        .data = esc1WireBuf,
    };
    for (attempt = 0; attempt < 5; attempt++) {
        if (!proto->readEEprom(esc, &ioMem)) {
            continue;
        }
        if (esc1WireBuf[0] == 0xFF || esc1WireBuf[0] == 0 ||
                esc1WireBuf[1] == 0xFF || esc1WireBuf[1] == 0 ||
                esc1WireBuf[2] == 0xFF || esc1WireBuf[2] == 0) {
            continue;
        }
        getBLHeliEscName(&ioMem, name);
        if (name[0] == '\0') {
            continue;
        }
        break;
    }
    if (attempt == 5) {
        return false;
    }
    esc1WireBuf[BLHELI_EEPROM_HEAD + eepromIndex] = value;
    for (attempt = 0; attempt < 5; attempt++) {
        ioMem.addr = 0;
        ioMem.len = 1;
        proto->eepromErase(esc, &ioMem);
        ioMem.addr = 0;
        ioMem.len = ESC_BUF_SIZE;
        ioMem.data = esc1WireBuf;
        if (!proto->writeEEprom(esc, &ioMem)) {
            continue;
        }
        ioMem.addr = 0;
        ioMem.data = esc1WireVerifyBuf;
        if (!proto->readEEprom(esc, &ioMem)) {
            continue;
        }
        if (memcmp(esc1WireVerifyBuf, esc1WireBuf, ESC_BUF_SIZE) != 0) {
            continue;
        }
        break;
    }
    if (attempt == 5) {
        return false;
    }
    return true;
}
int16_t esc1WireDumpEEprom(uint8_t escIndex, uint8_t** buf) {
    const esc1WireProtocol_t *escProto = escDevice[escIndex].protocol;
    if (escProto == NULL) {
        *buf = NULL;
        return BLHELI_BAD_PROTOCOL;
    }
    ioMem_t ioMem = {
        .addr = 0,
        .len = ESC_BUF_SIZE,
        .data = esc1WireBuf,
    };
    if (!escProto->readEEprom(&escHardware[escIndex], &ioMem)) {
        *buf = NULL;
        return BLHELI_ERROR;
    }
    *buf = esc1WireBuf;
    return ESC_BUF_SIZE;
}
uint16_t esc1WireGetSignature(uint8_t escIndex) {
    return escDevice[escIndex].signature;
}
uint16_t esc1WireGetVersion(uint8_t escIndex) {
    return escDevice[escIndex].version;
}
char* esc1WireGetMcuName(uint8_t escIndex){
    return escDevice[escIndex].name;
}
int16_t esc1WireGetParameter(uint8_t escIndex, const oneWireParameter_t *parameter) {
    escHardware_t *esc = &escHardware[escIndex];
    const esc1WireProtocol_t *proto = escDevice[escIndex].protocol;
    if (proto == NULL) {
        return -1;
    }
    const BLHeli_EEprom_t *layout = escDevice[escIndex].layout;
    if (layout == NULL) {
        return -1;
    }
    return readValueFromEEprom(esc, proto, *((uint8_t*)layout + parameter->offset));
}
int16_t esc1WireParameterFromDump(uint8_t escIndex, const oneWireParameter_t *parameter, uint8_t *buf) {
    const BLHeli_EEprom_t *layout = escDevice[escIndex].layout;
    if (layout == NULL) {
        return -1;
    }
    return buf[BLHELI_EEPROM_HEAD + *((uint8_t*)layout + parameter->offset)];
}
uint8_t esc1WireSetParameter(uint8_t escIndex, const oneWireParameter_t *parameter, uint8_t value) {
    escHardware_t *esc = &escHardware[escIndex];
    const esc1WireProtocol_t *proto = escDevice[escIndex].protocol;
    if (proto == NULL) {
        return false;
    }
    const BLHeli_EEprom_t *layout = escDevice[escIndex].layout;
    if (layout == NULL) {
        return false;
    }
    return writeValueToEEprom(esc, proto, *((uint8_t*)layout + parameter->offset), value);
}
const oneWireParameterValue_t motorOnOffParameterList[] = {
    {0x00, "disable"},
    {0x01, "enable"},
    {0, NULL},
};
const oneWireParameter_t motorEnableTxParameter = {
    .name = "txprogramming",
    .parameterNamed = motorOnOffParameterList,
    .offset = offsetof(BLHeli_EEprom_t, BL_ENABLE_TX),
};
const oneWireParameterValue_t motorBeaconDelayList[] = {
 {0x01, "1min"},
 {0x02, "2min"},
 {0x03, "5min"},
 {0x04, "10min"},
 {0x05, "off"},
 {0, NULL},
};
const oneWireParameter_t motorBeaconDelayParameter = {
 .name = "beacondelay",
 .parameterNamed = motorBeaconDelayList,
 .offset = offsetof(BLHeli_EEprom_t, BL_BEACON_DELAY)
};
const oneWireParameterNumerical_t motorBeaconStrengthValues = {
    .min = 0x00,
    .max = 0xFF,
    .offset = 0,
    .step = 1,
};
const oneWireParameter_t motorBeaconStrengthParameter = {
 .name = "beaconstrength",
 .parameterNamed = NULL,
    .parameterNumerical = &motorBeaconStrengthValues,
 .offset = offsetof(BLHeli_EEprom_t, BL_BEACON_STRENGTH)
};
const oneWireParameterNumerical_t motorBeepStrengthValues = {
    .min = 0x00,
    .max = 0xFF,
    .offset = 0,
    .step = 1,
};
const oneWireParameter_t motorBeepStrengthParameter = {
 .name = "beepstrength",
 .parameterNamed = NULL,
    .parameterNumerical = &motorBeepStrengthValues,
 .offset = offsetof(BLHeli_EEprom_t, BL_BEEP_STRENGTH)
};
const oneWireParameter_t motorBrakeOnStopParameter = {
    .name = "brakeonstop",
    .parameterNamed = motorOnOffParameterList,
    .offset = offsetof(BLHeli_EEprom_t, BL_BRAKE_ON_STOP)
};
const oneWireParameterValue_t motorDemagParameterList[] = {
    {0x01, "off"},
    {0x02, "low"},
    {0x03, "high"},
    {0, NULL},
};
const oneWireParameter_t motorDemagParameter = {
    .name = "demag",
    .parameterNamed = motorDemagParameterList,
    .offset = offsetof(BLHeli_EEprom_t, BL_DEMAG_COMP)
};
const oneWireParameterValue_t motorDirectionParameterList[] = {
    {0x01, "normal"},
    {0x02, "reversed"},
    {0x03, "bidirectional"},
    {0x04, "bidirectional reversed"},
    {0, NULL},
};
const oneWireParameter_t motorDirectionParameter = {
    .name = "direction",
    .parameterNamed = motorDirectionParameterList,
 .offset = offsetof(BLHeli_EEprom_t, BL_DIRECTION)
};
const oneWireParameterValue_t motorFrequencyParameterList[] = {
    {0x01, "high"},
    {0x02, "low"},
    {0x03, "damped light"},
    {0, NULL},
};
const oneWireParameter_t motorFrequencyParameter = {
    .name = "frequency",
    .parameterNamed = motorFrequencyParameterList,
 .offset = offsetof(BLHeli_EEprom_t, BL_PWM_FREQ)
};
const oneWireParameterNumerical_t motorThrottleValues = {
    .min = 0x00,
    .max = 0xFF,
    .offset = 1000,
    .step = 4,
};
const oneWireParameter_t motorMaxThrottleParameter = {
    .name = "maxthrottle",
    .parameterNamed = NULL,
    .parameterNumerical = &motorThrottleValues,
    .offset = offsetof(BLHeli_EEprom_t, BL_PPM_MAX_THROTLE)
};
const oneWireParameter_t motorMinThrottleParameter = {
    .name = "minthrottle",
    .parameterNamed = NULL,
    .parameterNumerical = &motorThrottleValues,
    .offset = offsetof(BLHeli_EEprom_t, BL_PPM_MIN_THROTLE)
};
const oneWireParameterValue_t motorStartupPowerParameterList[] = {
    {0x01, "0.031"},
    {0x02, "0.047"},
    {0x03, "0.063"},
    {0x04, "0.094"},
    {0x05, "0.125"},
    {0x06, "0.188"},
    {0x07, "0.25"},
    {0x08, "0.38"},
    {0x09, "0.50"},
    {0x0A, "0.75"},
    {0x0B, "1.00"},
    {0x0C, "1.25"},
    {0x0D, "1.50"},
    {0, NULL},
};
const oneWireParameter_t motorStartupPowerParameter = {
    .name = "startuppower",
    .parameterNamed = motorStartupPowerParameterList,
 .offset = offsetof(BLHeli_EEprom_t, BL_STARTUP_PWR)
};
const oneWireParameter_t motorTempProtectionParameter = {
    .name = "tempprotection",
    .parameterNamed = motorOnOffParameterList,
    .offset = offsetof(BLHeli_EEprom_t, BL_TEMP_PROTECTION)
};
const oneWireParameterValue_t motorTimingParameterList[] = {
    {0x01, "low"},
    {0x02, "medlow"},
    {0x03, "medium"},
    {0x04, "medhigh"},
    {0x05, "high"},
    {0, NULL},
};
const oneWireParameter_t motorTimingParameter = {
    .name = "timing",
    .parameterNamed = motorTimingParameterList,
 .offset = offsetof(BLHeli_EEprom_t, BL_COMM_TIMING)
};
#endif
