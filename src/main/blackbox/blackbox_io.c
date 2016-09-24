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

/*
 * printf a Blackbox header line with a leading "H " and trailing "\n" added automatically. blackboxHeaderBudget is
 * decreased to account for the number of bytes written.
 */
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

/**
 * Write an unsigned integer to the blackbox serial port using variable byte encoding.
 */
void blackboxWriteUnsignedVB(uint32_t value)
{
    
    while (value > 127) { 
        blackboxWrite((uint8_t) (value | 0x80)); 
        value >>= 7;
    }
    blackboxWrite(value);
}

/**
 * Write a signed integer to the blackbox serial port using ZigZig and variable byte encoding.
 */
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

/**
 * Write a 2 bit tag followed by 3 signed fields of 2, 4, 6 or 32 bits
 */
void blackboxWriteTag2_3S32(int32_t *values) {
    static const int NUM_FIELDS = 3;

    
    enum {
        BITS_2  = 0,
        BITS_4  = 1,
        BITS_6  = 2,
        BITS_32 = 3
    };

    enum {
        BYTES_1  = 0,
        BYTES_2  = 1,
        BYTES_3  = 2,
        BYTES_4  = 3
    };

    int x;
    int selector = BITS_2, selector2;

    /*
     * Find out how many bits the largest value requires to encode, and use it to choose one of the packing schemes
     * below:
     *
     * Selector possibilities
     *
     * 2 bits per field  ss11 2233,
     * 4 bits per field  ss00 1111 2222 3333
     * 6 bits per field  ss11 1111 0022 2222 0033 3333
     * 32 bits per field sstt tttt followed by fields of various byte counts
     */
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
            /*
             * Do another round to compute a selector for each field, assuming that they are at least 8 bits each
             *
             * Selector2 field possibilities
             * 0 - 8 bits
             * 1 - 16 bits
             * 2 - 24 bits
             * 3 - 32 bits
             */
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

/**
 * Write an 8-bit selector followed by four signed fields of size 0, 4, 8 or 16 bits.
 */
void blackboxWriteTag8_4S16(int32_t *values) {

    
    enum {
        FIELD_ZERO  = 0,
        FIELD_4BIT  = 1,
        FIELD_8BIT  = 2,
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

/**
 * Write `valueCount` fields from `values` to the Blackbox using signed variable byte encoding. A 1-byte header is
 * written first which specifies which fields are non-zero (so this encoding is compact when most fields are zero).
 *
 * valueCount must be 8 or less.
 */
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

/** Write unsigned integer **/
void blackboxWriteU32(int32_t value)
{
    blackboxWrite(value & 0xFF);
    blackboxWrite((value >> 8) & 0xFF);
    blackboxWrite((value >> 16) & 0xFF);
    blackboxWrite((value >> 24) & 0xFF);
}

/** Write float value in the integer form **/
void blackboxWriteFloat(float value)
{
    blackboxWriteU32(castFloatBytesToInt(value));
}

/**
 * If there is data waiting to be written to the blackbox device, attempt to write (a portion of) that now.
 * 
 * Returns true if all data has been flushed to the device.
 */
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

/**
 * Attempt to open the logging device. Returns true if successful.
 */
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
                    /*
                     * OpenLog's 230400 baud rate is very inaccurate, so it requires a larger inter-character gap in
                     * order to maintain synchronization.
                     */
                    portOptions |= SERIAL_STOPBITS_2;
                } else {
                    portOptions |= SERIAL_STOPBITS_1;
                }

                blackboxPort = openSerialPort(portConfig->identifier, FUNCTION_BLACKBOX, NULL, baudRates[baudRateIndex],
                    BLACKBOX_SERIAL_PORT_MODE, portOptions);

                /*
                 * The slowest MicroSD cards have a write latency approaching 150ms. The OpenLog's buffer is about 900
                 * bytes. In order for its buffer to be able to absorb this latency we must write slower than 6000 B/s.
                 *
                 * So:
                 *     Bytes per loop iteration = floor((looptime_ns / 1000000.0) * 6000)
                 *                              = floor((looptime_ns * 6000) / 1000000.0)
                 *                              = floor((looptime_ns * 3) / 500.0)
                 *                              = (looptime_ns * 3) / 500
                 */
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

/**
 * Close the Blackbox logging device immediately without attempting to flush any remaining data.
 */
void blackboxDeviceClose(void)
{
    switch (masterConfig.blackbox_device) {
        case BLACKBOX_DEVICE_SERIAL:
            closeSerialPort(blackboxPort);
            blackboxPort = NULL;

            /*
             * Normally this would be handled by mw.c, but since we take an unknown amount
             * of time to shut down asynchronously, we're the only ones that know when to call it.
             */
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

/**
 * Call once every loop iteration in order to maintain the global blackboxHeaderBudget with the number of bytes we can
 * transmit this iteration.
 */
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

/**
 * You must call this function before attempting to write Blackbox header bytes to ensure that the write will not
 * cause buffers to overflow. The number of bytes you can write is capped by the blackboxHeaderBudget. Calling this
 * reservation function doesn't decrease blackboxHeaderBudget, so you must manually decrement that variable by the
 * number of bytes you actually wrote.
 *
 * When the Blackbox device is FlashFS, a successful return code guarantees that no data will be lost if you write that
 * many bytes to the device (i.e. FlashFS's buffers won't overflow).
 *
 * When the device is a serial port, a successful return code guarantees that RaceFlight's serial Tx buffer will not
 * overflow, and the outgoing bandwidth is likely to be small enough to give the OpenLog time to absorb MicroSD card
 * latency. However the OpenLog could still end up silently dropping data.
 *
 * Returns:
 *  BLACKBOX_RESERVE_SUCCESS - Upon success
 *  BLACKBOX_RESERVE_TEMPORARY_FAILURE - The buffer is currently too full to service the request, try again later
 *  BLACKBOX_RESERVE_PERMANENT_FAILURE - The buffer is too small to ever service this request
 */
blackboxBufferReserveStatus_e blackboxDeviceReserveBufferSpace(int32_t bytes)
{
    if (bytes <= blackboxHeaderBudget) {
        return BLACKBOX_RESERVE_SUCCESS;
    }

    
    switch (masterConfig.blackbox_device) {
        case BLACKBOX_DEVICE_SERIAL:
            /*
             * One byte of the tx buffer isn't available for user data (due to its circular list implementation),
             * hence the -1. Note that the USB VCP implementation doesn't use a buffer and has txBufferSize set to zero.
             */
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
                /*
                 * The write doesn't currently fit in the buffer, so try to make room for it. Our flushing here means
                 * that the Blackbox header writing code doesn't have to guess about the best time to ask flashfs to
                 * flush, and doesn't stall waiting for a flush that would otherwise not automatically be called.
                 */
                flashfsFlushAsync();
            }

            return BLACKBOX_RESERVE_TEMPORARY_FAILURE;
#endif

        default:
            return BLACKBOX_RESERVE_PERMANENT_FAILURE;
    }
}

