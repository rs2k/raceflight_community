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
       
#include "io/serial.h"
#include "drivers/serial.h"
#define MAX_MSP_PORT_COUNT 2
#define RF_BUF_SIZE 256
char rf_custom_in_buffer[RF_BUF_SIZE];
char rf_custom_out_buffer[RF_BUF_SIZE];
void mspInit(serialConfig_t *serialConfig);
void mspProcess(void);
void sendMspTelemetry(void);
void mspSetTelemetryPort(serialPort_t *mspTelemetryPort);
void mspAllocateSerialPorts(serialConfig_t *serialConfig);
void mspReleasePortIfAllocated(serialPort_t *serialPort);
void process_rf_custom_string(char *rf_custom_buffer);
