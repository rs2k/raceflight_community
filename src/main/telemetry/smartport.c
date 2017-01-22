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
#include <stdlib.h>
#include <math.h>
#include "platform.h"
#ifdef TELEMETRY
#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/serial.h"
#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/adc.h"
#include "drivers/light_led.h"
#include "rx/rx.h"
#include "rx/msp.h"
#include "io/beeper.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/gps.h"
#include "io/gimbal.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"
#include "flight/altitudehold.h"
#include "telemetry/telemetry.h"
#include "telemetry/smartport.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"
enum
{
    SPSTATE_UNINITIALIZED,
    SPSTATE_INITIALIZED,
    SPSTATE_WORKING,
    SPSTATE_TIMEDOUT,
};
enum
{
    FSSP_START_STOP = 0x7E,
    FSSP_DATA_FRAME = 0x10,
    FSSP_SENSOR_ID1 = 0x1B,
    FSSP_SENSOR_ID2 = 0x0D,
    FSSP_SENSOR_ID3 = 0x34,
    FSSP_SENSOR_ID4 = 0x67,
};
enum
{
    FSSP_DATAID_SPEED = 0x0830 ,
    FSSP_DATAID_VFAS = 0x0210 ,
    FSSP_DATAID_CURRENT = 0x0200 ,
    FSSP_DATAID_RPM = 0x050F ,
    FSSP_DATAID_ALTITUDE = 0x0100 ,
    FSSP_DATAID_FUEL = 0x0600 ,
    FSSP_DATAID_ADC1 = 0xF102 ,
    FSSP_DATAID_ADC2 = 0xF103 ,
    FSSP_DATAID_LATLONG = 0x0800 ,
    FSSP_DATAID_CAP_USED = 0x0600 ,
    FSSP_DATAID_VARIO = 0x0110 ,
    FSSP_DATAID_CELLS = 0x0300 ,
    FSSP_DATAID_CELLS_LAST = 0x030F ,
    FSSP_DATAID_HEADING = 0x0840 ,
    FSSP_DATAID_ACCX = 0x0700 ,
    FSSP_DATAID_ACCY = 0x0710 ,
    FSSP_DATAID_ACCZ = 0x0720 ,
    FSSP_DATAID_T1 = 0x0400 ,
    FSSP_DATAID_T2 = 0x0410 ,
    FSSP_DATAID_GPS_ALT = 0x0820 ,
};
const uint16_t frSkyDataIdTable[] = {
    FSSP_DATAID_SPEED ,
    FSSP_DATAID_VFAS ,
    FSSP_DATAID_CURRENT ,
    FSSP_DATAID_ALTITUDE ,
    FSSP_DATAID_FUEL ,
    FSSP_DATAID_LATLONG ,
    FSSP_DATAID_LATLONG ,
    FSSP_DATAID_VARIO ,
    FSSP_DATAID_HEADING ,
    FSSP_DATAID_ACCX ,
    FSSP_DATAID_ACCY ,
    FSSP_DATAID_ACCZ ,
    FSSP_DATAID_T1 ,
    FSSP_DATAID_T2 ,
    FSSP_DATAID_GPS_ALT ,
    0
};
#define __USE_C99_MATH 
#define SMARTPORT_BAUD 57600
#define SMARTPORT_UART_MODE MODE_RXTX
#define SMARTPORT_SERVICE_TIMEOUT_MS 1
#define SMARTPORT_NOT_CONNECTED_TIMEOUT_MS 7000
static serialPort_t *smartPortSerialPort = NULL;
static serialPortConfig_t *portConfig;
static telemetryConfig_t *telemetryConfig;
static bool smartPortTelemetryEnabled = false;
static portSharing_e smartPortPortSharing;
extern void serialInit(serialConfig_t *);
char smartPortState = SPSTATE_UNINITIALIZED;
static uint8_t smartPortHasRequest = 0;
static uint8_t smartPortIdCnt = 0;
static uint32_t smartPortLastRequestTime = 0;
static void smartPortDataReceive(uint16_t c)
{
    uint32_t now = millis();
    static uint8_t lastChar;
    if (lastChar == FSSP_START_STOP) {
        smartPortState = SPSTATE_WORKING;
        if (c == FSSP_SENSOR_ID1 && (serialRxBytesWaiting(smartPortSerialPort) == 0)) {
            smartPortLastRequestTime = now;
            smartPortHasRequest = 1;
        }
    }
    lastChar = c;
}
static void smartPortSendByte(uint8_t c, uint16_t *crcp)
{
    if (c == 0x7D || c == 0x7E) {
        serialWrite(smartPortSerialPort, 0x7D);
        c ^= 0x20;
    }
    serialWrite(smartPortSerialPort, c);
    if (crcp == NULL)
        return;
    uint16_t crc = *crcp;
    crc += c;
    crc += crc >> 8;
    crc &= 0x00FF;
    *crcp = crc;
}
static void smartPortSendPackage(uint16_t id, uint32_t val)
{
    uint16_t crc = 0;
    smartPortSendByte(FSSP_DATA_FRAME, &crc);
    uint8_t *u8p = (uint8_t*)&id;
    smartPortSendByte(u8p[0], &crc);
    smartPortSendByte(u8p[1], &crc);
    u8p = (uint8_t*)&val;
    smartPortSendByte(u8p[0], &crc);
    smartPortSendByte(u8p[1], &crc);
    smartPortSendByte(u8p[2], &crc);
    smartPortSendByte(u8p[3], &crc);
    smartPortSendByte(0xFF - (uint8_t)crc, NULL);
}
void initSmartPortTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_SMARTPORT);
    smartPortPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_SMARTPORT);
}
void freeSmartPortTelemetryPort(void)
{
    closeSerialPort(smartPortSerialPort);
    smartPortSerialPort = NULL;
    smartPortState = SPSTATE_UNINITIALIZED;
    smartPortTelemetryEnabled = false;
}
void configureSmartPortTelemetryPort(void)
{
    portOptions_t portOptions;
    if (!portConfig) {
        return;
    }
    portOptions = SERIAL_BIDIR;
    if (telemetryConfig->telemetry_inversion) {
        portOptions |= SERIAL_INVERTED;
    }
    smartPortSerialPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_SMARTPORT, NULL, SMARTPORT_BAUD, SMARTPORT_UART_MODE, portOptions);
    if (!smartPortSerialPort) {
        return;
    }
    smartPortState = SPSTATE_INITIALIZED;
    smartPortTelemetryEnabled = true;
    smartPortLastRequestTime = millis();
}
bool canSendSmartPortTelemetry(void)
{
    return smartPortSerialPort && (smartPortState == SPSTATE_INITIALIZED || smartPortState == SPSTATE_WORKING);
}
bool isSmartPortTimedOut(void)
{
    return smartPortState >= SPSTATE_TIMEDOUT;
}
void checkSmartPortTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(smartPortPortSharing);
    if (newTelemetryEnabledValue == smartPortTelemetryEnabled) {
        return;
    }
    if (newTelemetryEnabledValue)
        configureSmartPortTelemetryPort();
    else
        freeSmartPortTelemetryPort();
}
void handleSmartPortTelemetry(void)
{
    uint32_t smartPortLastServiceTime = millis();
    if (!smartPortTelemetryEnabled) {
        return;
    }
    if (!canSendSmartPortTelemetry()) {
        return;
    }
    while (serialRxBytesWaiting(smartPortSerialPort) > 0) {
        uint8_t c = serialRead(smartPortSerialPort);
        smartPortDataReceive(c);
    }
    uint32_t now = millis();
    if ((now - smartPortLastRequestTime) > SMARTPORT_NOT_CONNECTED_TIMEOUT_MS) {
        smartPortState = SPSTATE_TIMEDOUT;
        return;
    }
    while (smartPortHasRequest) {
        if ((millis() - smartPortLastServiceTime) > SMARTPORT_SERVICE_TIMEOUT_MS) {
            smartPortHasRequest = 0;
            return;
        }
        uint16_t id = frSkyDataIdTable[smartPortIdCnt];
        if (id == 0) {
            smartPortIdCnt = 0;
            id = frSkyDataIdTable[smartPortIdCnt];
        }
        smartPortIdCnt++;
        int32_t tmpi;
        static uint8_t t1Cnt = 0;
        switch(id) {
#ifdef GPS
            case FSSP_DATAID_SPEED :
                if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
                    uint32_t tmpui = (GPS_speed * 36 + 36 / 2) / 100;
                    smartPortSendPackage(id, tmpui);
                    smartPortHasRequest = 0;
                }
                break;
#endif
            case FSSP_DATAID_VFAS :
                if (feature(FEATURE_VBAT)) {
                    smartPortSendPackage(id, vbat * 10);
                    smartPortHasRequest = 0;
                }
                break;
            case FSSP_DATAID_CURRENT :
                if (feature(FEATURE_CURRENT_METER)) {
                    smartPortSendPackage(id, amperage / 10);
                    smartPortHasRequest = 0;
                }
                break;
            case FSSP_DATAID_ALTITUDE :
                if (sensors(SENSOR_BARO)) {
                    smartPortSendPackage(id, BaroAlt);
                    smartPortHasRequest = 0;
                }
                break;
            case FSSP_DATAID_FUEL :
                if (feature(FEATURE_CURRENT_METER)) {
                    smartPortSendPackage(id, mAhDrawn);
                    smartPortHasRequest = 0;
                }
                break;
#ifdef GPS
            case FSSP_DATAID_LATLONG :
                if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
                    uint32_t tmpui = 0;
                    if (smartPortIdCnt & 1) {
                        tmpui = abs(GPS_coord[LON]);
                        tmpui = (tmpui + tmpui / 2) / 25 | 0x80000000;
                        if (GPS_coord[LON] < 0) tmpui |= 0x40000000;
                    }
                    else {
                        tmpui = abs(GPS_coord[LAT]);
                        tmpui = (tmpui + tmpui / 2) / 25;
                        if (GPS_coord[LAT] < 0) tmpui |= 0x40000000;
                    }
                    smartPortSendPackage(id, tmpui);
                    smartPortHasRequest = 0;
                }
                break;
#endif
            case FSSP_DATAID_VARIO :
                if (sensors(SENSOR_BARO)) {
                    smartPortSendPackage(id, vario);
                    smartPortHasRequest = 0;
                }
                break;
            case FSSP_DATAID_HEADING :
                smartPortSendPackage(id, attitude.values.yaw * 10);
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_ACCX :
                smartPortSendPackage(id, accSmooth[X] / 44);
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_ACCY :
                smartPortSendPackage(id, accSmooth[Y] / 44);
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_ACCZ :
                smartPortSendPackage(id, accSmooth[Z] / 44);
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_T1 :
                t1Cnt++;
                if (t1Cnt >= 4) {
                    t1Cnt = 1;
                }
                tmpi = t1Cnt * 10000;
                if (ARMING_FLAG(OK_TO_ARM))
                    tmpi += 1;
                if (ARMING_FLAG(PREVENT_ARMING))
                    tmpi += 2;
                if (ARMING_FLAG(ARMED))
                    tmpi += 4;
                if (FLIGHT_MODE(ANGLE_MODE))
                    tmpi += 10;
                if (FLIGHT_MODE(HORIZON_MODE))
                    tmpi += 20;
                if (FLIGHT_MODE(UNUSED_MODE))
                    tmpi += 40;
                if (FLIGHT_MODE(PASSTHRU_MODE))
                    tmpi += 40;
                if (FLIGHT_MODE(MAG_MODE))
                    tmpi += 100;
                if (FLIGHT_MODE(BARO_MODE))
                    tmpi += 200;
                if (FLIGHT_MODE(SONAR_MODE))
                    tmpi += 400;
                if (FLIGHT_MODE(GPS_HOLD_MODE))
                    tmpi += 1000;
                if (FLIGHT_MODE(GPS_HOME_MODE))
                    tmpi += 2000;
                if (FLIGHT_MODE(HEADFREE_MODE))
                    tmpi += 4000;
                smartPortSendPackage(id, (uint32_t)tmpi);
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_T2 :
                if (sensors(SENSOR_GPS)) {
#ifdef GPS
                    smartPortSendPackage(id, (STATE(GPS_FIX) ? 1000 : 0) + (STATE(GPS_FIX_HOME) ? 2000 : 0) + GPS_numSat);
                    smartPortHasRequest = 0;
#endif
                }
                else if (feature(FEATURE_GPS)) {
                    smartPortSendPackage(id, 0);
                    smartPortHasRequest = 0;
                }
                break;
#ifdef GPS
            case FSSP_DATAID_GPS_ALT :
                if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
                    smartPortSendPackage(id, GPS_altitude * 100);
                    smartPortHasRequest = 0;
                }
                break;
#endif
            default:
                break;
        }
    }
}
#endif
