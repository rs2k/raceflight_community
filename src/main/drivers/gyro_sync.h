/*
 * gyro_sync.h
 *
 *  Created on: 3 aug. 2015
 *      Author: borisb
 */

#define INTERRUPT_WAIT_TIME 3

extern uint8_t mpuDividerDrops;
extern uint32_t targetLooptime;
extern uint32_t targetESCwritetime;

extern uint8_t ESCWriteDenominator;
extern uint8_t accDenominator;

void gyroUpdateSampleRate(uint8_t lpf);
