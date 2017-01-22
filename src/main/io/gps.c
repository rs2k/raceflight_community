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
#include <ctype.h>
#include <string.h>
#include <math.h>
#include "platform.h"
#include "build_config.h"
#include "debug.h"
#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "sensors/sensors.h"
#include "io/serial.h"
#include "io/display.h"
#include "io/gps.h"
#include "flight/gps_conversion.h"
#include "flight/pid.h"
#include "flight/navigation.h"
#include "config/config.h"
#include "config/runtime_config.h"
#ifdef GPS
#define LOG_ERROR '?'
#define LOG_IGNORED '!'
#define LOG_SKIPPED '>'
#define LOG_NMEA_GGA 'g'
#define LOG_NMEA_RMC 'r'
#define LOG_UBLOX_SOL 'O'
#define LOG_UBLOX_STATUS 'S'
#define LOG_UBLOX_SVINFO 'I'
#define LOG_UBLOX_POSLLH 'P'
#define LOG_UBLOX_VELNED 'V'
#define GPS_SV_MAXSATS 16
char gpsPacketLog[GPS_PACKET_LOG_ENTRY_COUNT];
static char *gpsPacketLogChar = gpsPacketLog;
int32_t GPS_coord[2];
uint8_t GPS_numSat;
uint16_t GPS_hdop = 9999;
uint32_t GPS_packetCount = 0;
uint32_t GPS_svInfoReceivedCount = 0;
uint8_t GPS_update = 0;
uint16_t GPS_altitude;
uint16_t GPS_speed;
uint16_t GPS_ground_course = 0;
uint8_t GPS_numCh;
uint8_t GPS_svinfo_chn[GPS_SV_MAXSATS];
uint8_t GPS_svinfo_svid[GPS_SV_MAXSATS];
uint8_t GPS_svinfo_quality[GPS_SV_MAXSATS];
uint8_t GPS_svinfo_cno[GPS_SV_MAXSATS];
static gpsConfig_t *gpsConfig;
#define GPS_TIMEOUT (2500)
#define GPS_INIT_ENTRIES (GPS_BAUDRATE_MAX + 1)
#define GPS_BAUDRATE_CHANGE_DELAY (200)
static serialConfig_t *serialConfig;
static serialPort_t *gpsPort;
typedef struct gpsInitData_s {
    uint8_t index;
    uint8_t baudrateIndex;
    const char *ubx;
    const char *mtk;
} gpsInitData_t;
static const gpsInitData_t gpsInitData[] = {
    { GPS_BAUDRATE_115200, BAUD_115200, "$PUBX,41,1,0003,0001,115200,0*1E\r\n", "$PMTK251,115200*1F\r\n" },
    { GPS_BAUDRATE_57600, BAUD_57600, "$PUBX,41,1,0003,0001,57600,0*2D\r\n", "$PMTK251,57600*2C\r\n" },
    { GPS_BAUDRATE_38400, BAUD_38400, "$PUBX,41,1,0003,0001,38400,0*26\r\n", "$PMTK251,38400*27\r\n" },
    { GPS_BAUDRATE_19200, BAUD_19200, "$PUBX,41,1,0003,0001,19200,0*23\r\n", "$PMTK251,19200*22\r\n" },
    { GPS_BAUDRATE_9600, BAUD_9600, "$PUBX,41,1,0003,0001,9600,0*16\r\n", "" }
};
#define GPS_INIT_DATA_ENTRY_COUNT (sizeof(gpsInitData) / sizeof(gpsInitData[0]))
#define DEFAULT_BAUD_RATE_INDEX 0
static const uint8_t ubloxInit[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xC2,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x05, 0x40, 0xA7,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67,
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A,
};
#define UBLOX_SBAS_MESSAGE_LENGTH 16
typedef struct ubloxSbas_s {
    sbasMode_e mode;
    uint8_t message[UBLOX_SBAS_MESSAGE_LENGTH];
} ubloxSbas_t;
static const ubloxSbas_t ubloxSbas[] = {
    { SBAS_AUTO, { 0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0xE5}},
    { SBAS_EGNOS, { 0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x51, 0x08, 0x00, 0x00, 0x8A, 0x41}},
    { SBAS_WAAS, { 0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x04, 0xE0, 0x04, 0x00, 0x19, 0x9D}},
    { SBAS_MSAS, { 0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x00, 0x02, 0x02, 0x00, 0x35, 0xEF}},
    { SBAS_GAGAN, { 0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x80, 0x01, 0x00, 0x00, 0xB2, 0xE8}}
};
typedef enum {
    GPS_UNKNOWN,
    GPS_INITIALIZING,
    GPS_CHANGE_BAUD,
    GPS_CONFIGURE,
    GPS_RECEIVING_DATA,
    GPS_LOST_COMMUNICATION,
} gpsState_e;
gpsData_t gpsData;
static void shiftPacketLog(void)
{
    uint32_t i;
    for (i = ARRAYLEN(gpsPacketLog) - 1; i > 0 ; i--) {
        gpsPacketLog[i] = gpsPacketLog[i-1];
    }
}
static void gpsNewData(uint16_t c);
static bool gpsNewFrameNMEA(char c);
static bool gpsNewFrameUBLOX(uint8_t data);
static void gpsSetState(gpsState_e state)
{
    gpsData.state = state;
    gpsData.state_position = 0;
    gpsData.state_ts = millis();
    gpsData.messageState = GPS_MESSAGE_STATE_IDLE;
}
void gpsInit(serialConfig_t *initialSerialConfig, gpsConfig_t *initialGpsConfig)
{
    serialConfig = initialSerialConfig;
    gpsData.baudrateIndex = 0;
    gpsData.errors = 0;
    gpsData.timeouts = 0;
    memset(gpsPacketLog, 0x00, sizeof(gpsPacketLog));
    gpsConfig = initialGpsConfig;
    gpsSetState(GPS_UNKNOWN);
    gpsData.lastMessage = millis();
    serialPortConfig_t *gpsPortConfig = findSerialPortConfig(FUNCTION_GPS);
    if (!gpsPortConfig) {
        featureClear(FEATURE_GPS);
        return;
    }
    while (gpsInitData[gpsData.baudrateIndex].baudrateIndex != gpsPortConfig->gps_baudrateIndex) {
        gpsData.baudrateIndex++;
        if (gpsData.baudrateIndex >= GPS_INIT_DATA_ENTRY_COUNT) {
            gpsData.baudrateIndex = DEFAULT_BAUD_RATE_INDEX;
            break;
        }
    }
    portMode_t mode = MODE_RXTX;
#if !defined(COLIBRI_RACE) && !defined(LUX_RACE)
    if (gpsConfig->provider == GPS_NMEA)
     mode &= ~MODE_TX;
#endif
    gpsPort = openSerialPort(gpsPortConfig->identifier, FUNCTION_GPS, NULL, gpsInitData[gpsData.baudrateIndex].baudrateIndex, mode, SERIAL_NOT_INVERTED);
    if (!gpsPort) {
        featureClear(FEATURE_GPS);
        return;
    }
    gpsSetState(GPS_INITIALIZING);
}
void gpsInitNmea(void)
{
#if defined(COLIBRI_RACE) || defined(LUX_RACE)
 uint32_t now;
#endif
    switch(gpsData.state) {
        case GPS_INITIALIZING:
#if defined(COLIBRI_RACE) || defined(LUX_RACE)
     now = millis();
     if (now - gpsData.state_ts < 1000)
      return;
     gpsData.state_ts = now;
     if (gpsData.state_position < 1) {
      serialSetBaudRate(gpsPort, 4800);
      gpsData.state_position++;
     } else if (gpsData.state_position < 2) {
      serialPrint(gpsPort, "$PSRF100,1,115200,8,1,0*05\r\n");
      gpsData.state_position++;
     } else {
      gpsSetState(GPS_CHANGE_BAUD);
     }
     break;
#endif
        case GPS_CHANGE_BAUD:
#if defined(COLIBRI_RACE) || defined(LUX_RACE)
     now = millis();
     if (now - gpsData.state_ts < 1000)
      return;
     gpsData.state_ts = now;
     if (gpsData.state_position < 1) {
      serialSetBaudRate(gpsPort, baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex]);
      gpsData.state_position++;
     } else if (gpsData.state_position < 2) {
      serialPrint(gpsPort, "$PSRF103,00,6,00,0*23\r\n");
      gpsData.state_position++;
     } else {
#else
            serialSetBaudRate(gpsPort, baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex]);
#endif
            gpsSetState(GPS_RECEIVING_DATA);
#if defined(COLIBRI_RACE) || defined(LUX_RACE)
     }
#endif
            break;
    }
}
void gpsInitUblox(void)
{
    uint32_t now;
    if (!isSerialTransmitBufferEmpty(gpsPort))
        return;
    switch (gpsData.state) {
        case GPS_INITIALIZING:
            now = millis();
            if (now - gpsData.state_ts < GPS_BAUDRATE_CHANGE_DELAY)
                return;
            if (gpsData.state_position < GPS_INIT_ENTRIES) {
                baudRate_e newBaudRateIndex = gpsInitData[gpsData.state_position].baudrateIndex;
                gpsData.state_ts = now;
                if (lookupBaudRateIndex(serialGetBaudRate(gpsPort)) != newBaudRateIndex) {
                    serialSetBaudRate(gpsPort, baudRates[newBaudRateIndex]);
                    return;
                }
                serialPrint(gpsPort, gpsInitData[gpsData.baudrateIndex].ubx);
                gpsData.state_position++;
            } else {
                gpsSetState(GPS_CHANGE_BAUD);
            }
            break;
        case GPS_CHANGE_BAUD:
            serialSetBaudRate(gpsPort, baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex]);
            gpsSetState(GPS_CONFIGURE);
            break;
        case GPS_CONFIGURE:
            if( gpsConfig->autoConfig == GPS_AUTOCONFIG_OFF ) {
                gpsSetState(GPS_RECEIVING_DATA);
                break;
            }
            if (gpsData.messageState == GPS_MESSAGE_STATE_IDLE) {
                gpsData.messageState++;
            }
            if (gpsData.messageState == GPS_MESSAGE_STATE_INIT) {
                if (gpsData.state_position < sizeof(ubloxInit)) {
                    serialWrite(gpsPort, ubloxInit[gpsData.state_position]);
                    gpsData.state_position++;
                } else {
                    gpsData.state_position = 0;
                    gpsData.messageState++;
                }
            }
            if (gpsData.messageState == GPS_MESSAGE_STATE_SBAS) {
                if (gpsData.state_position < UBLOX_SBAS_MESSAGE_LENGTH) {
                    serialWrite(gpsPort, ubloxSbas[gpsConfig->sbasMode].message[gpsData.state_position]);
                    gpsData.state_position++;
                } else {
                    gpsData.messageState++;
                }
            }
            if (gpsData.messageState >= GPS_MESSAGE_STATE_ENTRY_COUNT) {
                gpsSetState(GPS_RECEIVING_DATA);
            }
            break;
    }
}
void gpsInitHardware(void)
{
    switch (gpsConfig->provider) {
        case GPS_NMEA:
            gpsInitNmea();
            break;
        case GPS_UBLOX:
            gpsInitUblox();
            break;
    }
}
void gpsThread(void)
{
    if (gpsPort) {
        while (serialRxBytesWaiting(gpsPort))
            gpsNewData(serialRead(gpsPort));
    }
    switch (gpsData.state) {
        case GPS_UNKNOWN:
            break;
        case GPS_INITIALIZING:
        case GPS_CHANGE_BAUD:
        case GPS_CONFIGURE:
            gpsInitHardware();
            break;
        case GPS_LOST_COMMUNICATION:
            gpsData.timeouts++;
            if (gpsConfig->autoBaud) {
                gpsData.baudrateIndex++;
                gpsData.baudrateIndex %= GPS_INIT_ENTRIES;
            }
            gpsData.lastMessage = millis();
            GPS_numSat = 0;
            DISABLE_STATE(GPS_FIX);
            gpsSetState(GPS_INITIALIZING);
            break;
        case GPS_RECEIVING_DATA:
            if (millis() - gpsData.lastMessage > GPS_TIMEOUT) {
                sensorsClear(SENSOR_GPS);
                gpsSetState(GPS_LOST_COMMUNICATION);
            }
            break;
    }
}
static void gpsNewData(uint16_t c)
{
    if (!gpsNewFrame(c)) {
        return;
    }
    gpsData.lastLastMessage = gpsData.lastMessage;
    gpsData.lastMessage = millis();
    sensorsSet(SENSOR_GPS);
    if (GPS_update == 1)
        GPS_update = 0;
    else
        GPS_update = 1;
#if 0
    debug[3] = GPS_update;
#endif
    onGpsNewData();
}
bool gpsNewFrame(uint8_t c)
{
    switch (gpsConfig->provider) {
        case GPS_NMEA:
            return gpsNewFrameNMEA(c);
        case GPS_UBLOX:
            return gpsNewFrameUBLOX(c);
    }
    return false;
}
#define NO_FRAME 0
#define FRAME_GGA 1
#define FRAME_RMC 2
#define FRAME_GSV 3
static uint32_t grab_fields(char *src, uint8_t mult)
{
    uint32_t i;
    uint32_t tmp = 0;
    for (i = 0; src[i] != 0; i++) {
        if (src[i] == '.') {
            i++;
            if (mult == 0)
                break;
            else
                src[i + mult] = 0;
        }
        tmp *= 10;
        if (src[i] >= '0' && src[i] <= '9')
            tmp += src[i] - '0';
        if (i >= 15)
            return 0;
    }
    return tmp;
}
typedef struct gpsDataNmea_s {
    int32_t latitude;
    int32_t longitude;
    uint8_t numSat;
    uint16_t altitude;
    uint16_t speed;
    uint16_t ground_course;
} gpsDataNmea_t;
static bool gpsNewFrameNMEA(char c)
{
    static gpsDataNmea_t gps_Msg;
    uint8_t frameOK = 0;
    static uint8_t param = 0, offset = 0, parity = 0;
    static char string[15];
    static uint8_t checksum_param, gps_frame = NO_FRAME;
    static uint8_t svMessageNum = 0;
    uint8_t svSatNum = 0, svPacketIdx = 0, svSatParam = 0;
    switch (c) {
        case '$':
            param = 0;
            offset = 0;
            parity = 0;
            break;
        case ',':
        case '*':
            string[offset] = 0;
            if (param == 0) {
                gps_frame = NO_FRAME;
                if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A')
                    gps_frame = FRAME_GGA;
                if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C')
                    gps_frame = FRAME_RMC;
                if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'S' && string[4] == 'V')
                    gps_frame = FRAME_GSV;
            }
            switch (gps_frame) {
                case FRAME_GGA:
                    switch(param) {
                        case 2:
                            gps_Msg.latitude = GPS_coord_to_degrees(string);
                            break;
                        case 3:
                            if (string[0] == 'S')
                                gps_Msg.latitude *= -1;
                            break;
                        case 4:
                            gps_Msg.longitude = GPS_coord_to_degrees(string);
                            break;
                        case 5:
                            if (string[0] == 'W')
                                gps_Msg.longitude *= -1;
                            break;
                        case 6:
                            if (string[0] > '0') {
                                ENABLE_STATE(GPS_FIX);
                            } else {
                                DISABLE_STATE(GPS_FIX);
                            }
                            break;
                        case 7:
                            gps_Msg.numSat = grab_fields(string, 0);
                            break;
                        case 9:
                            gps_Msg.altitude = grab_fields(string, 0);
                            break;
                    }
                    break;
                case FRAME_RMC:
                    switch(param) {
                        case 7:
                            gps_Msg.speed = ((grab_fields(string, 1) * 5144L) / 1000L);
                            break;
                        case 8:
                            gps_Msg.ground_course = (grab_fields(string, 1));
                            break;
                    }
                    break;
                case FRAME_GSV:
                    switch(param) {
                        case 2:
                            svMessageNum = grab_fields(string, 0);
                            break;
                        case 3:
                            GPS_numCh = grab_fields(string, 0);
                            break;
                    }
                    if(param < 4)
                        break;
                    svPacketIdx = (param - 4) / 4 + 1;
                    svSatNum = svPacketIdx + (4 * (svMessageNum - 1));
                    svSatParam = param - 3 - (4 * (svPacketIdx - 1));
                    if(svSatNum > GPS_SV_MAXSATS)
                        break;
                    switch(svSatParam) {
                        case 1:
                            GPS_svinfo_chn[svSatNum - 1] = svSatNum;
                            GPS_svinfo_svid[svSatNum - 1] = grab_fields(string, 0);
                            break;
                        case 4:
                            GPS_svinfo_cno[svSatNum - 1] = grab_fields(string, 0);
                            GPS_svinfo_quality[svSatNum - 1] = 0;
                            break;
                    }
                    GPS_svInfoReceivedCount++;
                    break;
            }
            param++;
            offset = 0;
            if (c == '*')
                checksum_param = 1;
            else
                parity ^= c;
            break;
        case '\r':
        case '\n':
            if (checksum_param) {
                shiftPacketLog();
                uint8_t checksum = 16 * ((string[0] >= 'A') ? string[0] - 'A' + 10 : string[0] - '0') + ((string[1] >= 'A') ? string[1] - 'A' + 10 : string[1] - '0');
                if (checksum == parity) {
                    *gpsPacketLogChar = LOG_IGNORED;
                    GPS_packetCount++;
                    switch (gps_frame) {
                    case FRAME_GGA:
                      *gpsPacketLogChar = LOG_NMEA_GGA;
                      frameOK = 1;
                      if (STATE(GPS_FIX)) {
                            GPS_coord[LAT] = gps_Msg.latitude;
                            GPS_coord[LON] = gps_Msg.longitude;
                            GPS_numSat = gps_Msg.numSat;
                            GPS_altitude = gps_Msg.altitude;
                        }
                        break;
                    case FRAME_RMC:
                        *gpsPacketLogChar = LOG_NMEA_RMC;
                        GPS_speed = gps_Msg.speed;
                        GPS_ground_course = gps_Msg.ground_course;
                        break;
                    }
                } else {
                    *gpsPacketLogChar = LOG_ERROR;
                }
            }
            checksum_param = 0;
            break;
        default:
            if (offset < 15)
                string[offset++] = c;
            if (!checksum_param)
                parity ^= c;
    }
    return frameOK;
}
typedef struct {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
} ubx_header;
typedef struct {
    uint32_t time;
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
} ubx_nav_posllh;
typedef struct {
    uint32_t time;
    uint8_t fix_type;
    uint8_t fix_status;
    uint8_t differential_status;
    uint8_t res;
    uint32_t time_to_first_fix;
    uint32_t uptime;
} ubx_nav_status;
typedef struct {
    uint32_t time;
    int32_t time_nsec;
    int16_t week;
    uint8_t fix_type;
    uint8_t fix_status;
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    uint32_t position_accuracy_3d;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t speed_accuracy;
    uint16_t position_DOP;
    uint8_t res;
    uint8_t satellites;
    uint32_t res2;
} ubx_nav_solution;
typedef struct {
    uint32_t time;
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
} ubx_nav_velned;
typedef struct {
    uint8_t chn;
    uint8_t svid;
    uint8_t flags;
    uint8_t quality;
    uint8_t cno;
    uint8_t elev;
    int16_t azim;
    int32_t prRes;
} ubx_nav_svinfo_channel;
typedef struct {
    uint32_t time;
    uint8_t numCh;
    uint8_t globalFlags;
    uint16_t reserved2;
    ubx_nav_svinfo_channel channel[16];
} ubx_nav_svinfo;
enum {
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_SOL = 0x6,
    MSG_VELNED = 0x12,
    MSG_SVINFO = 0x30,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_SET_RATE = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24
} ubx_protocol_bytes;
enum {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
} ubs_nav_fix_type;
enum {
    NAV_STATUS_FIX_VALID = 1
} ubx_nav_status_bits;
static uint8_t _ck_a;
static uint8_t _ck_b;
static bool _skip_packet;
static uint8_t _step;
static uint8_t _msg_id;
static uint16_t _payload_length;
static uint16_t _payload_counter;
static bool next_fix;
static uint8_t _class;
static bool _new_position;
static bool _new_speed;
#define UBLOX_PAYLOAD_SIZE 344
static union {
    ubx_nav_posllh posllh;
    ubx_nav_status status;
    ubx_nav_solution solution;
    ubx_nav_velned velned;
    ubx_nav_svinfo svinfo;
    uint8_t bytes[UBLOX_PAYLOAD_SIZE];
} _buffer;
void _update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    while (len--) {
        *ck_a += *data;
        *ck_b += *ck_a;
        data++;
    }
}
static bool UBLOX_parse_gps(void)
{
    uint32_t i;
    *gpsPacketLogChar = LOG_IGNORED;
    switch (_msg_id) {
    case MSG_POSLLH:
        *gpsPacketLogChar = LOG_UBLOX_POSLLH;
        GPS_coord[LON] = _buffer.posllh.longitude;
        GPS_coord[LAT] = _buffer.posllh.latitude;
        GPS_altitude = _buffer.posllh.altitude_msl / 10 / 100;
        if (next_fix) {
            ENABLE_STATE(GPS_FIX);
        } else {
            DISABLE_STATE(GPS_FIX);
        }
        _new_position = true;
        break;
    case MSG_STATUS:
        *gpsPacketLogChar = LOG_UBLOX_STATUS;
        next_fix = (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D);
        if (!next_fix)
            DISABLE_STATE(GPS_FIX);
        break;
    case MSG_SOL:
        *gpsPacketLogChar = LOG_UBLOX_SOL;
        next_fix = (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
        if (!next_fix)
            DISABLE_STATE(GPS_FIX);
        GPS_numSat = _buffer.solution.satellites;
        GPS_hdop = _buffer.solution.position_DOP;
        break;
    case MSG_VELNED:
        *gpsPacketLogChar = LOG_UBLOX_VELNED;
        GPS_speed = _buffer.velned.speed_2d;
        GPS_ground_course = (uint16_t) (_buffer.velned.heading_2d / 10000);
        _new_speed = true;
        break;
    case MSG_SVINFO:
        *gpsPacketLogChar = LOG_UBLOX_SVINFO;
        GPS_numCh = _buffer.svinfo.numCh;
        if (GPS_numCh > 16)
            GPS_numCh = 16;
        for (i = 0; i < GPS_numCh; i++){
            GPS_svinfo_chn[i]= _buffer.svinfo.channel[i].chn;
            GPS_svinfo_svid[i]= _buffer.svinfo.channel[i].svid;
            GPS_svinfo_quality[i]=_buffer.svinfo.channel[i].quality;
            GPS_svinfo_cno[i]= _buffer.svinfo.channel[i].cno;
        }
        GPS_svInfoReceivedCount++;
        break;
    default:
        return false;
    }
    if (_new_position && _new_speed) {
        _new_speed = _new_position = false;
        return true;
    }
    return false;
}
static bool gpsNewFrameUBLOX(uint8_t data)
{
    bool parsed = false;
    switch (_step) {
        case 0:
            if (PREAMBLE1 == data) {
                _skip_packet = false;
                _step++;
            }
            break;
        case 1:
            if (PREAMBLE2 != data) {
                _step = 0;
                break;
            }
            _step++;
            break;
        case 2:
            _step++;
            _class = data;
            _ck_b = _ck_a = data;
            break;
        case 3:
            _step++;
            _ck_b += (_ck_a += data);
            _msg_id = data;
            break;
        case 4:
            _step++;
            _ck_b += (_ck_a += data);
            _payload_length = data;
            break;
        case 5:
            _step++;
            _ck_b += (_ck_a += data);
            _payload_length += (uint16_t)(data << 8);
            if (_payload_length > UBLOX_PAYLOAD_SIZE) {
                _skip_packet = true;
            }
            _payload_counter = 0;
            if (_payload_length == 0) {
                _step = 7;
            }
            break;
        case 6:
            _ck_b += (_ck_a += data);
            if (_payload_counter < UBLOX_PAYLOAD_SIZE) {
                _buffer.bytes[_payload_counter] = data;
            }
            if (++_payload_counter >= _payload_length) {
                _step++;
            }
            break;
        case 7:
            _step++;
            if (_ck_a != data) {
                _skip_packet = true;
                gpsData.errors++;
            }
            break;
        case 8:
            _step = 0;
            shiftPacketLog();
            if (_ck_b != data) {
                *gpsPacketLogChar = LOG_ERROR;
                gpsData.errors++;
                break;
            }
            GPS_packetCount++;
            if (_skip_packet) {
                *gpsPacketLogChar = LOG_SKIPPED;
                break;
            }
            if (UBLOX_parse_gps()) {
                parsed = true;
            }
    }
    return parsed;
}
void gpsEnablePassthrough(serialPort_t *gpsPassthroughPort)
{
    waitForSerialPortToFinishTransmitting(gpsPort);
    waitForSerialPortToFinishTransmitting(gpsPassthroughPort);
    if(!(gpsPort->mode & MODE_TX))
        serialSetMode(gpsPort, gpsPort->mode | MODE_TX);
    LED0_OFF;
    LED1_OFF;
#ifdef DISPLAY
    if (feature(FEATURE_DISPLAY)) {
        displayShowFixedPage(PAGE_GPS);
    }
#endif
    char c;
    while(1) {
        if (serialRxBytesWaiting(gpsPort)) {
            LED0_ON;
            c = serialRead(gpsPort);
            gpsNewData(c);
            serialWrite(gpsPassthroughPort, c);
            LED0_OFF;
        }
        if (serialRxBytesWaiting(gpsPassthroughPort)) {
            LED1_ON;
            serialWrite(gpsPort, serialRead(gpsPassthroughPort));
            LED1_OFF;
        }
#ifdef DISPLAY
        if (feature(FEATURE_DISPLAY)) {
            updateDisplay();
        }
#endif
    }
}
void updateGpsIndicator(uint32_t currentTime)
{
    static uint32_t GPSLEDTime;
    if ((int32_t)(currentTime - GPSLEDTime) >= 0 && (GPS_numSat >= 5)) {
        GPSLEDTime = currentTime + 150000;
        LED1_TOGGLE;
    }
}
#endif
