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
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "blackbox_io.h"
#include "version.h"
#include "build_config.h"
#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/encoding.h"
#include "drivers/gpio.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/compass.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/gyro_sync.h"
#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "io/beeper.h"
#include "io/display.h"
#include "io/escservo.h"
#include "rx/rx.h"
#include "io/rc_controls.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"
#include "rx/msp.h"
#include "telemetry/telemetry.h"
#include "common/printf.h"
#include "flight/mixer.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/navigation.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"
#include "io/flashfs.h"
#define BLACKBOX_SERIAL_PORT_MODE MODE_TX
static uint8_t blackboxMaxHeaderBytesPerIteration;
int32_t blackboxHeaderBudget;
static serialPort_t *blackboxPort = NULL;
static portSharing_e blackboxPortSharing;
void blackboxWrite(uint8_t value)
{
    switch (masterConfig.blackbox_device) {
#ifdef USE_FLASHFS
        case BLACKBOX_DEVICE_FLASH:
            flashfsWriteByte(value);
        break;
#endif
        case BLACKBOX_DEVICE_SERIAL:
        default:
            serialWrite(blackboxPort, value);
        break;
    }
}
static void _putc(void *p, char c)
{
    (void)p;
    blackboxWrite(c);
}
static int blackboxPrintfv(const char *fmt, va_list va)
{
    return tfp_format(NULL, _putc, fmt, va);
}
int blackboxPrintf(const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    int written = blackboxPrintfv(fmt, va);
    va_end(va);
    return written;
}
void blackboxPrintfHeaderLine(const char *fmt, ...)
{
    va_list va;
    blackboxWrite('H');
    blackboxWrite(' ');
    va_start(va, fmt);
    int written = blackboxPrintfv(fmt, va);
    va_end(va);
    blackboxWrite('\n');
    blackboxHeaderBudget -= written + 3;
}
int blackboxPrint(const char *s)
{
    int length;
    const uint8_t *pos;
    switch (masterConfig.blackbox_device) {
#ifdef USE_FLASHFS
        case BLACKBOX_DEVICE_FLASH:
            length = strlen(s);
            flashfsWrite((const uint8_t*) s, length, false);
        break;
#endif
        case BLACKBOX_DEVICE_SERIAL:
        default:
            pos = (uint8_t*) s;
            while (*pos) {
                serialWrite(blackboxPort, *pos);
                pos++;
            }
            length = pos - (uint8_t*) s;
        break;
    }
    return length;
}
void blackboxWriteUnsignedVB(uint32_t value)
{
    while (value > 127) {
        blackboxWrite((uint8_t) (value | 0x80));
        value >>= 7;
    }
    blackboxWrite(value);
}
void blackboxWriteSignedVB(int32_t value)
{
    blackboxWriteUnsignedVB(zigzagEncode(value));
}
void blackboxWriteSignedVBArray(int32_t *array, int count)
{
    for (int i = 0; i < count; i++) {
        blackboxWriteSignedVB(array[i]);
    }
}
void blackboxWriteSigned16VBArray(int16_t *array, int count)
{
    for (int i = 0; i < count; i++) {
        blackboxWriteSignedVB(array[i]);
    }
}
void blackboxWriteS16(int16_t value)
{
    blackboxWrite(value & 0xFF);
    blackboxWrite((value >> 8) & 0xFF);
}
void blackboxWriteTag2_3S32(int32_t *values) {
    static const int NUM_FIELDS = 3;
    enum {
        BITS_2 = 0,
        BITS_4 = 1,
        BITS_6 = 2,
        BITS_32 = 3
    };
    enum {
        BYTES_1 = 0,
        BYTES_2 = 1,
        BYTES_3 = 2,
        BYTES_4 = 3
    };
    int x;
    int selector = BITS_2, selector2;
    for (x = 0; x < NUM_FIELDS; x++) {
        if (values[x] >= 32 || values[x] < -32) {
            selector = BITS_32;
            break;
        }
        if (values[x] >= 8 || values[x] < -8) {
             if (selector < BITS_6) {
                 selector = BITS_6;
             }
        } else if (values[x] >= 2 || values[x] < -2) {
            if (selector < BITS_4) {
                selector = BITS_4;
            }
        }
    }
    switch (selector) {
        case BITS_2:
            blackboxWrite((selector << 6) | ((values[0] & 0x03) << 4) | ((values[1] & 0x03) << 2) | (values[2] & 0x03));
        break;
        case BITS_4:
            blackboxWrite((selector << 6) | (values[0] & 0x0F));
            blackboxWrite((values[1] << 4) | (values[2] & 0x0F));
        break;
        case BITS_6:
            blackboxWrite((selector << 6) | (values[0] & 0x3F));
            blackboxWrite((uint8_t)values[1]);
            blackboxWrite((uint8_t)values[2]);
        break;
        case BITS_32:
            selector2 = 0;
            for (x = NUM_FIELDS - 1; x >= 0; x--) {
                selector2 <<= 2;
                if (values[x] < 128 && values[x] >= -128) {
                    selector2 |= BYTES_1;
                } else if (values[x] < 32768 && values[x] >= -32768) {
                    selector2 |= BYTES_2;
                } else if (values[x] < 8388608 && values[x] >= -8388608) {
                    selector2 |= BYTES_3;
                } else {
                    selector2 |= BYTES_4;
                }
            }
            blackboxWrite((selector << 6) | selector2);
            for (x = 0; x < NUM_FIELDS; x++, selector2 >>= 2) {
                switch (selector2 & 0x03) {
                    case BYTES_1:
                        blackboxWrite(values[x]);
                    break;
                    case BYTES_2:
                        blackboxWrite(values[x]);
                        blackboxWrite(values[x] >> 8);
                    break;
                    case BYTES_3:
                        blackboxWrite(values[x]);
                        blackboxWrite(values[x] >> 8);
                        blackboxWrite(values[x] >> 16);
                    break;
                    case BYTES_4:
                        blackboxWrite(values[x]);
                        blackboxWrite(values[x] >> 8);
                        blackboxWrite(values[x] >> 16);
                        blackboxWrite(values[x] >> 24);
                    break;
                }
            }
        break;
    }
}
void blackboxWriteTag8_4S16(int32_t *values) {
    enum {
        FIELD_ZERO = 0,
        FIELD_4BIT = 1,
        FIELD_8BIT = 2,
        FIELD_16BIT = 3
    };
    uint8_t selector, buffer;
    int nibbleIndex;
    int x;
    selector = 0;
    for (x = 3; x >= 0; x--) {
        selector <<= 2;
        if (values[x] == 0) {
            selector |= FIELD_ZERO;
        } else if (values[x] < 8 && values[x] >= -8) {
            selector |= FIELD_4BIT;
        } else if (values[x] < 128 && values[x] >= -128) {
            selector |= FIELD_8BIT;
        } else {
            selector |= FIELD_16BIT;
        }
    }
    blackboxWrite(selector);
    nibbleIndex = 0;
    buffer = 0;
    for (x = 0; x < 4; x++, selector >>= 2) {
        switch (selector & 0x03) {
            case FIELD_ZERO:
            break;
            case FIELD_4BIT:
                if (nibbleIndex == 0) {
                    buffer = values[x] << 4;
                    nibbleIndex = 1;
                } else {
                    blackboxWrite(buffer | (values[x] & 0x0F));
                    nibbleIndex = 0;
                }
            break;
            case FIELD_8BIT:
                if (nibbleIndex == 0) {
                    blackboxWrite(values[x]);
                } else {
                    blackboxWrite(buffer | ((values[x] >> 4) & 0x0F));
                    buffer = values[x] << 4;
                }
            break;
            case FIELD_16BIT:
                if (nibbleIndex == 0) {
                    blackboxWrite(values[x] >> 8);
                    blackboxWrite(values[x]);
                } else {
                    blackboxWrite(buffer | ((values[x] >> 12) & 0x0F));
                    blackboxWrite(values[x] >> 4);
                    buffer = values[x] << 4;
                }
            break;
        }
    }
    if (nibbleIndex == 1) {
        blackboxWrite(buffer);
    }
}
void blackboxWriteTag8_8SVB(int32_t *values, int valueCount)
{
    uint8_t header;
    int i;
    if (valueCount > 0) {
        if (valueCount == 1) {
            blackboxWriteSignedVB(values[0]);
        } else {
            header = 0;
            for (i = valueCount - 1; i >= 0; i--) {
                header <<= 1;
                if (values[i] != 0) {
                    header |= 0x01;
                }
            }
            blackboxWrite(header);
            for (i = 0; i < valueCount; i++) {
                if (values[i] != 0) {
                    blackboxWriteSignedVB(values[i]);
                }
            }
        }
    }
}
void blackboxWriteU32(int32_t value)
{
    blackboxWrite(value & 0xFF);
    blackboxWrite((value >> 8) & 0xFF);
    blackboxWrite((value >> 16) & 0xFF);
    blackboxWrite((value >> 24) & 0xFF);
}
void blackboxWriteFloat(float value)
{
    blackboxWriteU32(castFloatBytesToInt(value));
}
bool blackboxDeviceFlush(void)
{
    switch (masterConfig.blackbox_device) {
        case BLACKBOX_DEVICE_SERIAL:
            return isSerialTransmitBufferEmpty(blackboxPort);
#ifdef USE_FLASHFS
        case BLACKBOX_DEVICE_FLASH:
            return flashfsFlushAsync();
#endif
        default:
            return false;
    }
}
bool blackboxDeviceOpen(void)
{
    switch (masterConfig.blackbox_device) {
        case BLACKBOX_DEVICE_SERIAL:
            {
                serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_BLACKBOX);
                baudRate_e baudRateIndex;
                portOptions_t portOptions = SERIAL_PARITY_NO | SERIAL_NOT_INVERTED;
                if (!portConfig) {
                    return false;
                }
                blackboxPortSharing = determinePortSharing(portConfig, FUNCTION_BLACKBOX);
                baudRateIndex = portConfig->blackbox_baudrateIndex;
                if (baudRates[baudRateIndex] == 230400) {
                    portOptions |= SERIAL_STOPBITS_2;
                } else {
                    portOptions |= SERIAL_STOPBITS_1;
                }
                blackboxPort = openSerialPort(portConfig->identifier, FUNCTION_BLACKBOX, NULL, baudRates[baudRateIndex],
                    BLACKBOX_SERIAL_PORT_MODE, portOptions);
                blackboxMaxHeaderBytesPerIteration = constrain((targetESCwritetime * 3) / 500, 1, BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION);
                return blackboxPort != NULL;
            }
            break;
#ifdef USE_FLASHFS
        case BLACKBOX_DEVICE_FLASH:
            if (flashfsGetSize() == 0 || isBlackboxDeviceFull()) {
                return false;
            }
            blackboxMaxHeaderBytesPerIteration = BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION;
            return true;
        break;
#endif
        default:
            return false;
    }
}
void blackboxDeviceClose(void)
{
    switch (masterConfig.blackbox_device) {
        case BLACKBOX_DEVICE_SERIAL:
            closeSerialPort(blackboxPort);
            blackboxPort = NULL;
            if (blackboxPortSharing == PORTSHARING_SHARED) {
                mspAllocateSerialPorts(&masterConfig.serialConfig);
            }
            break;
#ifdef USE_FLASHFS
        case BLACKBOX_DEVICE_FLASH:
            break;
#endif
    }
}
bool isBlackboxDeviceFull(void)
{
    switch (masterConfig.blackbox_device) {
        case BLACKBOX_DEVICE_SERIAL:
            return false;
#ifdef USE_FLASHFS
        case BLACKBOX_DEVICE_FLASH:
            return flashfsIsEOF();
#endif
        default:
            return false;
    }
}
void blackboxReplenishHeaderBudget()
{
    int32_t freeSpace;
    switch (masterConfig.blackbox_device) {
        case BLACKBOX_DEVICE_SERIAL:
            freeSpace = serialTxBytesFree(blackboxPort);
        break;
#ifdef USE_FLASHFS
        case BLACKBOX_DEVICE_FLASH:
            freeSpace = flashfsGetWriteBufferFreeSpace();
        break;
#endif
        default:
            freeSpace = 0;
    }
    blackboxHeaderBudget = MIN(MIN(freeSpace, blackboxHeaderBudget + blackboxMaxHeaderBytesPerIteration), BLACKBOX_MAX_ACCUMULATED_HEADER_BUDGET);
}
blackboxBufferReserveStatus_e blackboxDeviceReserveBufferSpace(int32_t bytes)
{
    if (bytes <= blackboxHeaderBudget) {
        return BLACKBOX_RESERVE_SUCCESS;
    }
    switch (masterConfig.blackbox_device) {
        case BLACKBOX_DEVICE_SERIAL:
            if (blackboxPort->txBufferSize && bytes > (int32_t) blackboxPort->txBufferSize - 1) {
                return BLACKBOX_RESERVE_PERMANENT_FAILURE;
            }
            return BLACKBOX_RESERVE_TEMPORARY_FAILURE;
#ifdef USE_FLASHFS
        case BLACKBOX_DEVICE_FLASH:
            if (bytes > (int32_t) flashfsGetWriteBufferSize()) {
                return BLACKBOX_RESERVE_PERMANENT_FAILURE;
            }
            if (bytes > (int32_t) flashfsGetWriteBufferFreeSpace()) {
                flashfsFlushAsync();
            }
            return BLACKBOX_RESERVE_TEMPORARY_FAILURE;
#endif
        default:
            return BLACKBOX_RESERVE_PERMANENT_FAILURE;
    }
}
