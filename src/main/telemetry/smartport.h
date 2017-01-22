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
       
#ifndef TELEMETRY_SMARTPORT_H_
#define TELEMETRY_SMARTPORT_H_ 
void initSmartPortTelemetry(telemetryConfig_t *);
void handleSmartPortTelemetry(void);
void checkSmartPortTelemetryState(void);
void configureSmartPortTelemetryPort(void);
void freeSmartPortTelemetryPort(void);
bool isSmartPortTimedOut(void);
#endif
