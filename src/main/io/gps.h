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
       
#define LAT 0
#define LON 1
#define GPS_DEGREES_DIVIDER 10000000L
typedef enum {
    GPS_NMEA = 0,
    GPS_UBLOX
} gpsProvider_e;
#define GPS_PROVIDER_MAX GPS_UBLOX
typedef enum {
    SBAS_AUTO = 0,
    SBAS_EGNOS,
    SBAS_WAAS,
    SBAS_MSAS,
    SBAS_GAGAN
} sbasMode_e;
#define SBAS_MODE_MAX SBAS_GAGAN
typedef enum {
    GPS_BAUDRATE_115200 = 0,
    GPS_BAUDRATE_57600,
    GPS_BAUDRATE_38400,
    GPS_BAUDRATE_19200,
    GPS_BAUDRATE_9600
} gpsBaudRate_e;
typedef enum {
    GPS_AUTOCONFIG_OFF = 0,
    GPS_AUTOCONFIG_ON,
} gpsAutoConfig_e;
typedef enum {
    GPS_AUTOBAUD_OFF = 0,
    GPS_AUTOBAUD_ON
} gpsAutoBaud_e;
#define GPS_BAUDRATE_MAX GPS_BAUDRATE_9600
typedef struct gpsConfig_s {
    gpsProvider_e provider;
    sbasMode_e sbasMode;
    gpsAutoConfig_e autoConfig;
    gpsAutoBaud_e autoBaud;
} gpsConfig_t;
typedef struct gpsCoordinateDDDMMmmmm_s {
    int16_t dddmm;
    int16_t mmmm;
} gpsCoordinateDDDMMmmmm_t;
typedef enum {
    GPS_MESSAGE_STATE_IDLE = 0,
    GPS_MESSAGE_STATE_INIT,
    GPS_MESSAGE_STATE_SBAS,
    GPS_MESSAGE_STATE_MAX = GPS_MESSAGE_STATE_SBAS
} gpsMessageState_e;
#define GPS_MESSAGE_STATE_ENTRY_COUNT (GPS_MESSAGE_STATE_MAX + 1)
typedef struct gpsData_s {
    uint8_t state;
    uint8_t baudrateIndex;
    uint32_t errors;
    uint32_t timeouts;
    uint32_t lastMessage;
    uint32_t lastLastMessage;
    uint32_t state_position;
    uint32_t state_ts;
    gpsMessageState_e messageState;
} gpsData_t;
#define GPS_PACKET_LOG_ENTRY_COUNT 21
extern char gpsPacketLog[GPS_PACKET_LOG_ENTRY_COUNT];
extern gpsData_t gpsData;
extern int32_t GPS_coord[2];
extern uint8_t GPS_numSat;
extern uint16_t GPS_hdop;
extern uint8_t GPS_update;
extern uint32_t GPS_packetCount;
extern uint32_t GPS_svInfoReceivedCount;
extern uint16_t GPS_altitude;
extern uint16_t GPS_speed;
extern uint16_t GPS_ground_course;
extern uint8_t GPS_numCh;
extern uint8_t GPS_svinfo_chn[16];
extern uint8_t GPS_svinfo_svid[16];
extern uint8_t GPS_svinfo_quality[16];
extern uint8_t GPS_svinfo_cno[16];
#define GPS_DBHZ_MIN 0
#define GPS_DBHZ_MAX 55
void gpsThread(void);
bool gpsNewFrame(uint8_t c);
void updateGpsIndicator(uint32_t currentTime);
