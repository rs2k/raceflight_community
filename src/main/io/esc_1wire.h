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
typedef struct {
    uint8_t min;
    uint8_t max;
    int16_t step;
    int16_t offset;
} oneWireParameterNumerical_t;
typedef struct {
    uint8_t value;
    const char *name;
} oneWireParameterValue_t;
typedef struct {
    const char* name;
    const oneWireParameterValue_t *parameterNamed;
    const oneWireParameterNumerical_t *parameterNumerical;
    uint32_t offset;
} oneWireParameter_t;
extern const oneWireParameter_t motorBeaconDelayParameter;
extern const oneWireParameter_t motorBeaconStrengthParameter;
extern const oneWireParameter_t motorBeepStrengthParameter;
extern const oneWireParameter_t motorBrakeOnStopParameter;
extern const oneWireParameter_t motorDemagParameter;
extern const oneWireParameter_t motorDirectionParameter;
extern const oneWireParameter_t motorEnableTxParameter;
extern const oneWireParameter_t motorFrequencyParameter;
extern const oneWireParameter_t motorMaxThrottleParameter;
extern const oneWireParameter_t motorMinThrottleParameter;
extern const oneWireParameter_t motorStartupPowerParameter;
extern const oneWireParameter_t motorTempProtectionParameter;
extern const oneWireParameter_t motorTimingParameter;
int16_t esc1WireGetParameter(uint8_t escIndex, const oneWireParameter_t *layout);
uint8_t esc1WireSetParameter(uint8_t escIndex, const oneWireParameter_t *layout, uint8_t value);
int16_t esc1WireParameterFromDump(uint8_t escIndex, const oneWireParameter_t *parameter, uint8_t *buf);
int8_t esc1WireInitialize(void);
void esc1WireDisconnect(void);
void esc1WireRelease(void);
int8_t esc1WireStatus(void);
int8_t esc1WireCheckMotor(uint8_t escIndex);
uint8_t esc1WireInitFlash(uint8_t escIndex);
uint8_t esc1WireFlash(uint8_t escIndex, uint8_t *data, uint16_t address, uint16_t length);
#ifdef ESC_HEX
const uint8_t* esc1WireFindHex(char *name);
uint16_t esc1WireGetHexVersion(const uint8_t *binary);
uint8_t esc1WireGetHexMcuName(const uint8_t *binary, char *name, uint8_t len);
#endif
#define BLHELI_ERROR -5
#define BLHELI_NOT_STARTED -4
#define BLHELI_NO_CONNECTION -3
#define BLHELI_BAD_PROTOCOL -2
#define BLHELI_BAD_LAYOUT -1
#define BLHELI_OK 0
int16_t esc1WireDumpEEprom(uint8_t escIndex, uint8_t **buf);
uint16_t esc1WireGetSignature(uint8_t escIndex);
uint16_t esc1WireGetVersion(uint8_t escIndex);
char* esc1WireGetMcuName(uint8_t escIndex);
#endif
