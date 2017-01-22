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
       
#include "common/utils.h"
#define DEFIO_TAG(pinid) CONCAT(DEFIO_TAG__, pinid)
#define DEFIO_TAG__NONE 0
#define DEFIO_REC(pinid) CONCAT(DEFIO_REC__, pinid)
#define DEFIO_REC__NONE NULL
#define DEFIO_IO(pinid) (IO_t)DEFIO_REC(pinid)
#define DEFIO_REC_INDEXED(idx) (ioRecs + (idx))
#define DEFIO_TAG_MAKE(gpioid,pin) ((((gpioid) + 1) << 4) | (pin))
#define DEFIO_TAG_ISEMPTY(tag) (!(tag))
#define DEFIO_TAG_GPIOID(tag) (((tag) >> 4) - 1)
#define DEFIO_TAG_PIN(tag) ((tag) & 0x0f)
#include "target.h"
#include "io_def_generated.h"
