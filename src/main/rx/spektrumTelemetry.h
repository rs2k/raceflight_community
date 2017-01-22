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
       
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"
#include "spektrumTelemetrySensors.h"
#pragma pack(1)
#define SPEKTRUM_SRXL_ID 0xA5
#define SRXL_TELEM_ID 0x80
#define SRXL_TELEM_LENGTH 21
#define SRXL_BIND_ID 0x41
#define SRXL_BIND_LENGTH 19
#define SRXL_BIND_ENTER 0xEB
#define SRXL_BIND_REQUEST_STATUS 0xB5
#define SRXL_BIND_BOUND_DATA_REPORT 0xDB
#define SRXL_BIND_SET_BIND 0x5B
#define SRXL_POLY 0x1021
#define MAX_SENSORS 8
typedef enum
{
 TELEM_START = 0,
 TELEM_FLIGHTLOG = TELEM_START,
 TELEM_INTERNAL,
 TELEM_XBUS,
 NUM_TELEM_STATES
} TELEMETRY_STATE;
#define INTERNAL_ID 0x7E
typedef struct
{
 uint8_t id;
 uint8_t rfu;
 uint16_t rpm;
 uint16_t packVoltage;
 uint16_t temperature;
 uint8_t spare[8];
} TLM_INTERNAL;
#define FLIGHTLOG_ID 0x7F
typedef struct
{
 uint8_t id;
 uint8_t rfu;
 uint16_t A;
 uint16_t B;
 uint16_t L;
 uint16_t R;
 uint16_t F;
 uint16_t H;
 uint16_t rxVoltage;
} TLM_FLIGHTLOG;
typedef struct
{
 uint8_t data[16];
} TLM_XBUS;
#define TLM_XBUS_LEN (sizeof(TLM_XBUS))
typedef struct
{
 uint8_t data[16];
} TLM_SERIAL;
#define TLM_SERIAL_LEN (sizeof(TLM_SERIAL))
typedef union
{
 TLM_FLIGHTLOG flightLog;
 TLM_INTERNAL internalSensors;
 TLM_XBUS xBUS;
 TLM_SERIAL serial;
 uint8_t raw[16];
} TELEMETRY_STR;
typedef union
{
 struct PACKET
 {
  uint8_t SRXL_ID;
  uint8_t identifier;
  uint8_t length;
  TELEMETRY_STR data;
  uint16_t crc;
 }packet;
 uint8_t raw[21];
} STR_SRXL_TELEM;
#define SRXLTELEM_PACKET_LEN (sizeof(STR_SRXL_TELEM))
typedef union
{
 struct
 {
  uint8_t info;
  uint8_t holds;
  uint16_t frames;
  uint16_t fades[4];
  uint8_t rfu[4];
 };
 uint8_t asBytes[16];
} STR_FLIGHT_LOG;
STR_FLIGHT_LOG flightLog;
#define FLIGHT_LOG_LEN (sizeof(STR_FLIGHT_LOG))
typedef union
{
 struct
 {
  uint8_t request;
  uint64_t guid;
  uint8_t type;
  uint32_t rfID;
 };
 uint8_t raw[14];
} STR_BIND;
#define SRXLBIND_PAYLOAD_LEN (sizeof(STR_BIND))
typedef struct
{
 uint8_t srxlID;
 uint8_t subID;
 uint8_t length;
 STR_BIND data;
 uint16_t crc;
} STR_SRXL_BIND;
#define SRXLBIND_PACKET_LEN (sizeof(STR_SRXL_BIND))
typedef enum
{
 REMOTE_A,
 REMOTE_B,
 REMOTE_L,
 REMOTE_R
} RF_REMOTES;
struct
{
 uint8_t sensorPosition;
 uint8_t textLine;
 uint8_t sensorCount;
 uint8_t sensorAddress[MAX_SENSORS];
} xbus;
void sendSpektrumSRXL(uint32_t baseAddress, uint8_t packetSize);
void sendSpektrumTelem(void);
void sendSpektrumBind(void);
uint16_t srxlCrc16(uint16_t crc, uint8_t data, uint16_t poly);
