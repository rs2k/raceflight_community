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
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "barometer.h"

#include "gpio.h"
#include "system.h"
#include "bus_i2c.h"

#include "build_config.h"

#ifndef MS5611_I2C_INSTANCE
#define MS5611_I2C_INSTANCE I2C_DEVICE
#endif


#ifndef MS5611_ADDR
#define MS5611_ADDR                 0x77
#endif
#define CMD_RESET               0x1E 
#define CMD_ADC_READ            0x00 
#define CMD_ADC_CONV            0x40 
#define CMD_ADC_D1              0x00 
#define CMD_ADC_D2              0x10 
#define CMD_ADC_256             0x00 
#define CMD_ADC_512             0x02 
#define CMD_ADC_1024            0x04 
#define CMD_ADC_2048            0x06 
#define CMD_ADC_4096            0x08 
#define CMD_PROM_RD             0xA0 
#define PROM_NB                 8

static void ms5611_reset(void);
static uint16_t ms5611_prom(int8_t coef_num);
STATIC_UNIT_TESTED int8_t ms5611_crc(uint16_t *prom);
static uint32_t ms5611_read_adc(void);
static void ms5611_start_ut(void);
static void ms5611_get_ut(void);
static void ms5611_start_up(void);
static void ms5611_get_up(void);
STATIC_UNIT_TESTED void ms5611_calculate(int32_t *pressure, int32_t *temperature);

STATIC_UNIT_TESTED uint32_t ms5611_ut;  
STATIC_UNIT_TESTED uint32_t ms5611_up;  
STATIC_UNIT_TESTED uint16_t ms5611_c[PROM_NB];  
static uint8_t ms5611_osr = CMD_ADC_4096;

bool ms5611Detect(baro_t *baro)
{
    bool ack = false;
    uint8_t sig;
    int i;

    delay(10); 

    ack = i2cRead(MS5611_I2C_INSTANCE, MS5611_ADDR, CMD_PROM_RD, 1, &sig);
    if (!ack)
        return false;

    ms5611_reset();
    
    for (i = 0; i < PROM_NB; i++)
        ms5611_c[i] = ms5611_prom(i);
    
    if (ms5611_crc(ms5611_c) != 0)
        return false;

    
    baro->ut_delay = 10000;
    baro->up_delay = 10000;
    baro->start_ut = ms5611_start_ut;
    baro->get_ut = ms5611_get_ut;
    baro->start_up = ms5611_start_up;
    baro->get_up = ms5611_get_up;
    baro->calculate = ms5611_calculate;

    return true;
}

static void ms5611_reset(void)
{
    i2cWrite(MS5611_I2C_INSTANCE, MS5611_ADDR, CMD_RESET, 1);
    delayMicroseconds(2800);
}

static uint16_t ms5611_prom(int8_t coef_num)
{
    uint8_t rxbuf[2] = { 0, 0 };
    i2cRead(MS5611_I2C_INSTANCE, MS5611_ADDR, CMD_PROM_RD + coef_num * 2, 2, rxbuf); 
    return rxbuf[0] << 8 | rxbuf[1];
}

STATIC_UNIT_TESTED int8_t ms5611_crc(uint16_t *prom)
{
    int32_t i, j;
    uint32_t res = 0;
    uint8_t crc = prom[7] & 0xF;
    prom[7] &= 0xFF00;

    bool blankEeprom = true;

    for (i = 0; i < 16; i++) {
        if (prom[i >> 1]) {
            blankEeprom = false;
        }
        if (i & 1)
            res ^= ((prom[i >> 1]) & 0x00FF);
        else
            res ^= (prom[i >> 1] >> 8);
        for (j = 8; j > 0; j--) {
            if (res & 0x8000)
                res ^= 0x1800;
            res <<= 1;
        }
    }
    prom[7] |= crc;
    if (!blankEeprom && crc == ((res >> 12) & 0xF))
        return 0;

    return -1;
}

static uint32_t ms5611_read_adc(void)
{
    uint8_t rxbuf[3];
    i2cRead(MS5611_I2C_INSTANCE, MS5611_ADDR, CMD_ADC_READ, 3, rxbuf); 
    return (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}

static void ms5611_start_ut(void)
{
    i2cWrite(MS5611_I2C_INSTANCE, MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr, 1); 
}

static void ms5611_get_ut(void)
{
    ms5611_ut = ms5611_read_adc();
}

static void ms5611_start_up(void)
{
    i2cWrite(MS5611_I2C_INSTANCE, MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr, 1); 
}

static void ms5611_get_up(void)
{
    ms5611_up = ms5611_read_adc();
}

STATIC_UNIT_TESTED void ms5611_calculate(int32_t *pressure, int32_t *temperature)
{
    uint32_t press;
    int64_t temp;
    int64_t delt;
    int64_t dT = (int64_t)ms5611_ut - ((uint64_t)ms5611_c[5] * 256);
    int64_t off = ((int64_t)ms5611_c[2] << 16) + (((int64_t)ms5611_c[4] * dT) >> 7);
    int64_t sens = ((int64_t)ms5611_c[1] << 15) + (((int64_t)ms5611_c[3] * dT) >> 8);
    temp = 2000 + ((dT * (int64_t)ms5611_c[6]) >> 23);

    if (temp < 2000) { 
        delt = temp - 2000;
        delt = 5 * delt * delt;
        off -= delt >> 1;
        sens -= delt >> 2;
        if (temp < -1500) { 
            delt = temp + 1500;
            delt = delt * delt;
            off -= 7 * delt;
            sens -= (11 * delt) >> 1;
        }
    temp -= ((dT * dT) >> 31);
    }
    press = ((((int64_t)ms5611_up * sens) >> 21) - off) >> 15;


    if (pressure)
        *pressure = press;
    if (temperature)
        *temperature = temp;
}
