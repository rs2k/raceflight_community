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
       
#ifndef __TFP_PRINTF__
#define __TFP_PRINTF__ 
#include <stdarg.h>
#include "drivers/serial.h"
void init_printf(void *putp, void (*putf) (void *, char));
int tfp_printf(const char *fmt, ...);
int tfp_sprintf(char *s, const char *fmt, ...);
int tfp_format(void *putp, void (*putf) (void *, char), const char *fmt, va_list va);
#define printf tfp_printf
#define sprintf tfp_sprintf
void setPrintfSerialPort(serialPort_t *serialPort);
#endif
