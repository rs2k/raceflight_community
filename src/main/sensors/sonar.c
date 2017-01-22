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
#include <math.h>
#include "platform.h"
#include "build_config.h"
#include "common/maths.h"
#include "common/axis.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/gpio.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/sonar.h"
float baro_cf_vel;
float baro_cf_alt;
#ifdef SONAR
int16_t sonarMaxRangeCm;
int16_t sonarMaxAltWithTiltCm;
int16_t sonarCfAltCm;
STATIC_UNIT_TESTED int16_t sonarMaxTiltDeciDegrees;
float sonarMaxTiltCos;
static int32_t calculatedAltitude;
const sonarHardware_t *sonarGetHardwareConfiguration(batteryConfig_t *batteryConfig)
{
#if defined(NAZE) || defined(EUSTM32F103RC) || defined(PORT103R)
 static const sonarHardware_t const sonarPWM56 = {
  .trigger_pin = Pin_8,
  .trigger_gpio = GPIOB,
  .echo_pin = Pin_9,
  .echo_gpio = GPIOB,
  .triggerIO = IO_TAG(PB8),
  .echoIO = IO_TAG(PB9),
 };
 static const sonarHardware_t sonarRC78 = {
  .trigger_pin = Pin_0,
  .trigger_gpio = GPIOB,
  .echo_pin = Pin_1,
  .echo_gpio = GPIOB,
  .triggerIO = IO_TAG(PB0),
  .echoIO = IO_TAG(PB1),
 };
 if (feature(FEATURE_SOFTSERIAL)
         || feature(FEATURE_RX_PARALLEL_PWM)
         || (feature(FEATURE_CURRENT_METER) && batteryConfig->currentMeterType == CURRENT_SENSOR_ADC)) {
  return &sonarPWM56;
 }
 else {
  return &sonarRC78;
 }
#elif defined(OLIMEXINO)
 UNUSED(batteryConfig);
 static const sonarHardware_t const sonarHardware = {
  .trigger_pin = Pin_0,
  .trigger_gpio = GPIOB,
  .echo_pin = Pin_1,
  .echo_gpio = GPIOB,
  .triggerIO = IO_TAG(PB0),
  .echoIO = IO_TAG(PB1),
 };
 return &sonarHardware;
#elif defined(CC3D)
 UNUSED(batteryConfig);
 static const sonarHardware_t const sonarHardware = {
  .trigger_pin = Pin_5,
  .trigger_gpio = GPIOB,
  .echo_pin = Pin_0,
  .echo_gpio = GPIOB,
  .triggerIO = IO_TAG(PB5),
  .echoIO = IO_TAG(PB0),
 };
 return &sonarHardware;
#elif defined(SPRACINGF3) || defined(SPRACINGF3MINI)
 UNUSED(batteryConfig);
 static const sonarHardware_t const sonarHardware = {
  .trigger_pin = Pin_0,
  .trigger_gpio = GPIOB,
  .echo_pin = Pin_1,
  .echo_gpio = GPIOB,
  .triggerIO = IO_TAG(PB0),
  .echoIO = IO_TAG(PB1),
 };
 return &sonarHardware;
#elif defined(SPARKY)
 UNUSED(batteryConfig);
 static const sonarHardware_t const sonarHardware = {
  .trigger_pin = Pin_2,
  .trigger_gpio = GPIOA,
  .echo_pin = Pin_1,
  .echo_gpio = GPIOB,
  .triggerIO = IO_TAG(PA2),
  .echoIO = IO_TAG(PB1),
 };
 return &sonarHardware;
#elif defined(UNIT_TEST)
 UNUSED(batteryConfig);
 return 0;
#else
#error Sonar not defined for target
#endif
}
void sonarInit(const sonarHardware_t *sonarHardware)
{
 sonarRange_t sonarRange;
 hcsr04_init(sonarHardware, &sonarRange);
 sensorsSet(SENSOR_SONAR);
 sonarMaxRangeCm = sonarRange.maxRangeCm;
 sonarCfAltCm = sonarMaxRangeCm / 2;
 sonarMaxTiltDeciDegrees = sonarRange.detectionConeExtendedDeciDegrees / 2;
 sonarMaxTiltCos = cos_approx(sonarMaxTiltDeciDegrees / 10.0f * RAD);
 sonarMaxAltWithTiltCm = sonarMaxRangeCm * sonarMaxTiltCos;
 calculatedAltitude = SONAR_OUT_OF_RANGE;
}
#define DISTANCE_SAMPLES_MEDIAN 5
static int32_t applySonarMedianFilter(int32_t newSonarReading)
{
 static int32_t sonarFilterSamples[DISTANCE_SAMPLES_MEDIAN];
 static int currentFilterSampleIndex = 0;
 static bool medianFilterReady = false;
 int nextSampleIndex;
 if (newSonarReading > SONAR_OUT_OF_RANGE)
 {
  nextSampleIndex = (currentFilterSampleIndex + 1);
  if (nextSampleIndex == DISTANCE_SAMPLES_MEDIAN) {
   nextSampleIndex = 0;
   medianFilterReady = true;
  }
  sonarFilterSamples[currentFilterSampleIndex] = newSonarReading;
  currentFilterSampleIndex = nextSampleIndex;
 }
 if (medianFilterReady)
  return quickMedianFilter5(sonarFilterSamples);
 else
  return newSonarReading;
}
void sonarUpdate(void)
{
 hcsr04_start_reading();
}
int32_t sonarRead(void)
{
 int32_t distance = hcsr04_get_distance();
 if (distance > HCSR04_MAX_RANGE_CM)
  distance = SONAR_OUT_OF_RANGE;
 return applySonarMedianFilter(distance);
}
int32_t sonarCalculateAltitude(int32_t sonarDistance, float cosTiltAngle)
{
 if (cosTiltAngle <= sonarMaxTiltCos)
  calculatedAltitude = SONAR_OUT_OF_RANGE;
 else
  calculatedAltitude = sonarDistance * cosTiltAngle;
 return calculatedAltitude;
}
int32_t sonarGetLatestAltitude(void)
{
 return calculatedAltitude;
}
#endif
