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
#include "platform.h"
#include "build_config.h"
#include "gpio.h"
#define MODE_OFFSET 0
#define PUPD_OFFSET 2
#define OUTPUT_OFFSET 4
#define MODE_MASK ((1|2))
#define PUPD_MASK ((1|2))
#define OUTPUT_MASK ((1|2))
void gpioInit(GPIO_TypeDef *gpio, gpio_config_t *config)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint32_t pinIndex;
    for (pinIndex = 0; pinIndex < 16; pinIndex++) {
        uint32_t pinMask = (0x1 << pinIndex);
        if (config->pin & pinMask) {
            GPIO_InitStructure.GPIO_Pin = pinMask;
            GPIO_InitStructure.GPIO_Mode = (config->mode >> MODE_OFFSET) & MODE_MASK;
            GPIOSpeed_TypeDef speed = GPIO_Medium_Speed;
            switch (config->speed) {
                case Speed_10MHz:
                    speed = GPIO_Medium_Speed;
                    break;
                case Speed_2MHz:
                    speed = GPIO_Low_Speed;
                    break;
                case Speed_50MHz:
                    speed = GPIO_Fast_Speed;
                    break;
            }
            GPIO_InitStructure.GPIO_Speed = speed;
            GPIO_InitStructure.GPIO_OType = (config->mode >> OUTPUT_OFFSET) & OUTPUT_MASK;
            GPIO_InitStructure.GPIO_PuPd = (config->mode >> PUPD_OFFSET) & PUPD_MASK;
            GPIO_Init(gpio, &GPIO_InitStructure);
        }
    }
}
void gpioExtiLineConfig(uint8_t portsrc, uint8_t pinsrc)
{
    SYSCFG_EXTILineConfig(portsrc, pinsrc);
}
