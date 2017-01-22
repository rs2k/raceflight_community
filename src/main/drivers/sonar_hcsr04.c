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
#include "system.h"
#include "gpio.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "sonar_hcsr04.h"
#if defined(SONAR)
STATIC_UNIT_TESTED volatile int32_t measurement = -1;
static uint32_t lastMeasurementAt;
static sonarHardware_t const *sonarHardware;
extiCallbackRec_t hcsr04_extiCallbackRec;
static IO_t echoIO;
void hcsr04_extiHandler(extiCallbackRec_t* cb)
{
    static uint32_t timing_start;
    uint32_t timing_stop;
    UNUSED(cb);
    if (digitalIn(sonarHardware->echo_gpio, sonarHardware->echo_pin) != 0) {
        timing_start = micros();
    }
    else {
        timing_stop = micros();
        if (timing_stop > timing_start) {
            measurement = timing_stop - timing_start;
        }
    }
}
void hcsr04_init(const sonarHardware_t *initialSonarHardware, sonarRange_t *sonarRange)
{
    sonarHardware = initialSonarHardware;
    sonarRange->maxRangeCm = HCSR04_MAX_RANGE_CM;
    sonarRange->detectionConeDeciDegrees = HCSR04_DETECTION_CONE_DECIDEGREES;
    sonarRange->detectionConeExtendedDeciDegrees = HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES;
#if !defined(UNIT_TEST)
    gpio_config_t gpio;
#ifdef STM32F303xC
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
#endif
    gpio.pin = sonarHardware->trigger_pin;
    gpio.mode = Mode_Out_PP;
    gpio.speed = Speed_2MHz;
    gpioInit(sonarHardware->trigger_gpio, &gpio);
    gpio.pin = sonarHardware->echo_pin;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(sonarHardware->echo_gpio, &gpio);
    echoIO = IOGetByTag(sonarHardware->echoIO);
#ifdef USE_EXTI
    EXTIHandlerInit(&hcsr04_extiCallbackRec, hcsr04_extiHandler);
    EXTIConfig(echoIO, &hcsr04_extiCallbackRec, NVIC_PRIO_SONAR_EXTI, EXTI_Trigger_Rising_Falling);
    EXTIEnable(echoIO, true);
#endif
    lastMeasurementAt = millis() - 60;
#else
    UNUSED(lastMeasurementAt);
#endif
}
void hcsr04_start_reading(void)
{
#if !defined(UNIT_TEST)
    uint32_t now = millis();
    if (now < (lastMeasurementAt + 60)) {
        return;
    }
    lastMeasurementAt = now;
    digitalHi(sonarHardware->trigger_gpio, sonarHardware->trigger_pin);
    delayMicroseconds(11);
    digitalLo(sonarHardware->trigger_gpio, sonarHardware->trigger_pin);
#endif
}
int32_t hcsr04_get_distance(void)
{
    int32_t distance = measurement / 59;
    return distance;
}
#endif
